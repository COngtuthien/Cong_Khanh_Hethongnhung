#include <systemc.h>

// --- MODULE VISION ON MCU ---
SC_MODULE(VisionMCU) {
    sc_in<uint32_t> color_input; 
    sc_out<uint32_t> object_type; 
    sc_in<bool> clk;

    void process() {
        while(true) {
            wait(); // Chờ cạnh lên của Clock
            uint32_t val = color_input.read();
            
            // GIẢ LẬP DELAY XỬ LÝ: MCU mất 2 chu kỳ (20ns) để nhận diện
            wait(2); 
            
            if (val == 1) {
                object_type.write(1); // Xuyên thấu là BALL
                cout << "@ " << sc_time_stamp() << ": MCU nhan dien ra BALL" << endl;
            } else if (val == 2) {
                object_type.write(2); // Xuyên thấu là BOX
                cout << "@ " << sc_time_stamp() << ": MCU nhan dien ra BOX" << endl;
            } else {
                object_type.write(0);
            }
        }
    }

    SC_CTOR(VisionMCU) {
        SC_CTHREAD(process, clk.pos());
    }
};

// --- MODULE SORTER ---
SC_MODULE(Sorter) {
    sc_in<uint32_t> object_type;
    sc_out<bool> servo_ball;
    sc_out<bool> servo_box;

    void control() {
        while(true) {
            uint32_t type = object_type.read();
            
            // GIẢ LẬP DELAY CƠ KHÍ: Cánh tay mất 10ns để phản ứng
            wait(10, SC_NS); 
            
            servo_ball.write(type == 1);
            servo_box.write(type == 2);
            
            wait(); // Đợi object_type thay đổi
        }
    }

    SC_CTOR(Sorter) {
        SC_THREAD(control);
        sensitive << object_type;
    }
};

// --- KỊCH BẢN MÔ PHỎNG (Sửa tại đây) ---
int sc_main(int argc, char* argv[]) {
    sc_clock clk("clk", 10, SC_NS);
    sc_signal<uint32_t> sig_input, sig_type;
    sc_signal<bool> sig_ball, sig_box;

    VisionMCU mcu("MCU_Block");
    mcu.clk(clk);
    mcu.color_input(sig_input);
    mcu.object_type(sig_type);

    Sorter sorter("Sorter_Block");
    sorter.object_type(sig_type);
    sorter.servo_ball(sig_ball);
    sorter.servo_box(sig_box);

    sc_trace_file *wf = sc_create_vcd_trace_file("waveform");
    sc_trace(wf, clk, "Clock");
    sc_trace(wf, sig_input, "Shape_Input");
    sc_trace(wf, sig_ball, "Servo_Ball_Output");
    sc_trace(wf, sig_box, "Servo_Box_Output");

    // --- CHẠY THỬ NGHIỆM ---
    cout << "--- Bat dau mo phong ---" << endl;
    
    // 1. Trang thai trong
    sig_input.write(0); sc_start(20, SC_NS);
    
    // 2. CHO BALL DI QUA
    sig_input.write(1); sc_start(60, SC_NS); 
    sig_input.write(0); sc_start(40, SC_NS);
    
    // 3. CHO BOX DI QUA (Day la phan ban thieu luc nay)
    sig_input.write(2); sc_start(60, SC_NS); 
    sig_input.write(0); sc_start(40, SC_NS);

    sc_close_vcd_trace_file(wf);
    cout << "--- Mo phong hoan tat! ---" << endl;
    return 0;
}
