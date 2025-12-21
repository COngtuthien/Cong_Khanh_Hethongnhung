#include <systemc>
#include <iostream>
#include <fstream>
#include <random>
#include <iomanip>
#include <string>

using namespace sc_core;

// -------------------- Data types --------------------
enum Label { L_NONE = 0, L_BALL = 1, L_BOX = 2 };

static const char* label_str(Label l) {
    switch (l) {
        case L_BALL: return "ball";
        case L_BOX:  return "box";
        default:     return "none";
    }
}

struct Sample {
    int   id = 0;
    Label gt = L_NONE;          // ground truth
};

struct AIOut {
    int   id = 0;
    Label gt = L_NONE;
    Label pred = L_NONE;        // predicted label from "AI"
    float conf = 0.0f;          // confidence [0..1]
};

// Required for sc_fifo printing
inline std::ostream& operator<<(std::ostream& os, const Sample& s) {
    os << "Sample{id=" << s.id << ",gt=" << label_str(s.gt) << "}";
    return os;
}
inline std::ostream& operator<<(std::ostream& os, const AIOut& o) {
    os << "AIOut{id=" << o.id << ",gt=" << label_str(o.gt)
       << ",pred=" << label_str(o.pred) << ",conf=" << o.conf << "}";
    return os;
}

// -------------------- Stimulus: generate ground-truth samples --------------------
SC_MODULE(Stimulus) {
    sc_fifo_out<Sample> out;

    int n_ball = 50;
    int n_box  = 50;

    SC_CTOR(Stimulus) {
        SC_THREAD(gen);
    }

    void gen() {
        int id = 0;
        for (int i = 0; i < n_ball; i++) {
            Sample s; s.id = id++; s.gt = L_BALL;
            out.write(s);
            wait(SC_ZERO_TIME); // no real time, just delta-cycle
        }
        for (int i = 0; i < n_box; i++) {
            Sample s; s.id = id++; s.gt = L_BOX;
            out.write(s);
            wait(SC_ZERO_TIME);
        }
    }
};

// -------------------- "AI model" (mock): output predicted label + confidence --------------------
SC_MODULE(AIModel) {
    sc_fifo_in<Sample> in;
    sc_fifo_out<AIOut> out;

    // You can tune these to resemble your real model behavior
    float p_correct_ball = 0.90f;  // P(pred==BALL | gt=BALL)
    float p_correct_box  = 0.90f;  // P(pred==BOX  | gt=BOX)
    float conf_correct_mu = 0.82f; // mean confidence when correct
    float conf_wrong_mu   = 0.55f; // mean confidence when wrong
    float conf_sigma      = 0.10f; // spread

    std::mt19937 rng;
    std::uniform_real_distribution<float> uni{0.0f, 1.0f};
    std::normal_distribution<float> norm_correct;
    std::normal_distribution<float> norm_wrong;

    SC_CTOR(AIModel)
        : rng(12345),
          norm_correct(conf_correct_mu, conf_sigma),
          norm_wrong(conf_wrong_mu, conf_sigma)
    {
        SC_THREAD(run);
    }

    static float clamp01(float x) {
        if (x < 0.f) return 0.f;
        if (x > 1.f) return 1.f;
        return x;
    }

    void run() {
        while (true) {
            Sample s = in.read();

            AIOut o;
            o.id = s.id;
            o.gt = s.gt;

            bool correct = false;
            if (s.gt == L_BALL) correct = (uni(rng) < p_correct_ball);
            else if (s.gt == L_BOX) correct = (uni(rng) < p_correct_box);

            if (correct) {
                o.pred = s.gt;
                o.conf = clamp01(norm_correct(rng));
            } else {
                // wrong prediction flips label
                o.pred = (s.gt == L_BALL) ? L_BOX : L_BALL;
                o.conf = clamp01(norm_wrong(rng));
            }

            out.write(o);
            wait(SC_ZERO_TIME);
        }
    }
};

// -------------------- Decision + Servo mapping --------------------
struct DecisionOut {
    int   id = 0;
    Label gt = L_NONE;
    Label pred = L_NONE;       // raw AI pred
    float conf = 0.0f;
    Label decided = L_NONE;    // after threshold gating
    int servo_angle = 90;      // mapped angle
};

inline std::ostream& operator<<(std::ostream& os, const DecisionOut& d) {
    os << "DecisionOut{id=" << d.id
       << ",gt=" << label_str(d.gt)
       << ",pred=" << label_str(d.pred)
       << ",conf=" << d.conf
       << ",decided=" << label_str(d.decided)
       << ",angle=" << d.servo_angle << "}";
    return os;
}

SC_MODULE(DecisionServo) {
    sc_fifo_in<AIOut> in;
    sc_fifo_out<DecisionOut> out;

    // match firmware-style config
    float threshold = 0.50f;
    int ball_angle = 45;
    int box_angle  = 0;
    int neutral_angle = 90;

    SC_CTOR(DecisionServo) {
        SC_THREAD(run);
    }

    void run() {
        while (true) {
            AIOut o = in.read();

            DecisionOut d;
            d.id = o.id;
            d.gt = o.gt;
            d.pred = o.pred;
            d.conf = o.conf;

            // threshold gating
            if (o.conf < threshold) {
                d.decided = L_NONE;
            } else {
                d.decided = o.pred;
            }

            // servo mapping
            if (d.decided == L_BALL) d.servo_angle = ball_angle;
            else if (d.decided == L_BOX) d.servo_angle = box_angle;
            else d.servo_angle = neutral_angle;

            out.write(d);
            wait(SC_ZERO_TIME);
        }
    }
};

// -------------------- Scoreboard: accuracy + angle correctness + confusion matrix --------------------
SC_MODULE(Scoreboard) {
    sc_fifo_in<DecisionOut> in;

    std::ofstream csv;

    int total = 0;
    int correct_decision = 0;      // decided label equals gt (gt ball/box only)
    int correct_angle = 0;         // servo angle matches expected for gt (ball->ball_angle, box->box_angle)
    int none_count = 0;            // decided none (below threshold)

    // confusion matrix over decided label vs gt (ball/box)
    // rows: gt (BALL, BOX)
    // cols: decided (BALL, BOX, NONE)
    int cm[2][3] = {{0,0,0},{0,0,0}};

    int expect_ball_angle = 45;
    int expect_box_angle = 0;

    int expected_angle_for(Label gt) {
        if (gt == L_BALL) return expect_ball_angle;
        if (gt == L_BOX)  return expect_box_angle;
        return 90;
    }

    SC_CTOR(Scoreboard) {
        csv.open("ai_servo_eval.csv");
        csv << "id,gt,pred,conf,decided,servo_angle,decision_correct,angle_correct\n";
        SC_THREAD(run);
    }

    ~Scoreboard() {
        if (csv.is_open()) csv.close();
    }

    void run() {
        while (true) {
            DecisionOut d = in.read();
            total++;

            bool decision_ok = (d.decided == d.gt);
            bool angle_ok = (d.servo_angle == expected_angle_for(d.gt));

            if (d.decided == L_NONE) none_count++;

            if (decision_ok) correct_decision++;
            if (angle_ok) correct_angle++;

            // update confusion (only for gt ball/box)
            int r = (d.gt == L_BALL) ? 0 : 1;
            int c = (d.decided == L_BALL) ? 0 : (d.decided == L_BOX) ? 1 : 2;
            cm[r][c]++;

            csv << d.id << "," << label_str(d.gt) << "," << label_str(d.pred) << ","
                << std::fixed << std::setprecision(3) << d.conf << ","
                << label_str(d.decided) << "," << d.servo_angle << ","
                << (decision_ok ? 1 : 0) << "," << (angle_ok ? 1 : 0) << "\n";

            wait(SC_ZERO_TIME);

            // Stop condition: when enough samples processed (example: 100)
            if (total >= 100) {
                std::cout << "\n=== SUMMARY ===\n";
                std::cout << "Total samples: " << total << "\n";
                std::cout << "Decision accuracy (decided==gt): "
                          << (100.0 * correct_decision / total) << " %\n";
                std::cout << "Angle correctness: "
                          << (100.0 * correct_angle / total) << " %\n";
                std::cout << "Decided NONE (below threshold): " << none_count << "\n\n";

                std::cout << "Confusion Matrix (GT rows x DECIDED cols)\n";
                std::cout << "          BALL    BOX    NONE\n";
                std::cout << "GT=BALL   " << std::setw(5) << cm[0][0]
                          << "  " << std::setw(5) << cm[0][1]
                          << "  " << std::setw(5) << cm[0][2] << "\n";
                std::cout << "GT=BOX    " << std::setw(5) << cm[1][0]
                          << "  " << std::setw(5) << cm[1][1]
                          << "  " << std::setw(5) << cm[1][2] << "\n";

                std::cout << "\nCSV saved: ai_servo_eval.csv\n";
                sc_stop();
            }
        }
    }
};

int sc_main(int argc, char** argv) {
    // Channels
    sc_fifo<Sample>      q0("q0", 16);
    sc_fifo<AIOut>       q1("q1", 16);
    sc_fifo<DecisionOut> q2("q2", 16);

    // Modules
    Stimulus     stim("stim");
    AIModel      ai("ai");
    DecisionServo ds("ds");
    Scoreboard   sb("sb");

    // Connect
    stim.out(q0);
    ai.in(q0); ai.out(q1);
    ds.in(q1); ds.out(q2);
    sb.in(q2);

    // Match firmware-ish config (edit if needed)
    ds.threshold = 0.50f;
    ds.ball_angle = 45;
    ds.box_angle  = 0;
    ds.neutral_angle = 90;

    sb.expect_ball_angle = 45;
    sb.expect_box_angle  = 0;

    // Run (no real-time)
    sc_start();

    return 0;
}