// Introduction to DSP and Filters
#include "mbed.h"
#include "arm_math.h"
#include "drivers/LCD_DISCO_F429ZI.h"

#define WINDOW_SIZE 10 // Example window size, adjust as needed

// Define Regs & Configurations --> Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Second configure to set the DPS // page 33
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define CTRL_REG3 0x22 // page 32
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define OUT_X_L 0x28

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

#define FILTER_COEFFICIENT 0.1f // Adjust this value as needed

// added by Scott
#define SAMPLES 64 // FFT size
#define FFT_BUFFER_SIZE 500 // needs to be at least SAMPLES * 2 + 1

// DigitalOut led(LED1);

float fftIn_x[FFT_BUFFER_SIZE];
float fftOut_x[FFT_BUFFER_SIZE];

float fftIn_y[FFT_BUFFER_SIZE];
float fftOut_y[FFT_BUFFER_SIZE];

float fftIn_z[FFT_BUFFER_SIZE];
float fftOut_z[FFT_BUFFER_SIZE];

arm_rfft_fast_instance_f32 fftHandler;

// uint8_t fftFlag = 0;

// EventFlags object declaration
EventFlags flags;
static BufferedSerial serial_port(USBTX, USBRX);

// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}


// data ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}



int main() {
    serial_port.set_baud(9600);
    //spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    //interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);
    
    //spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers --> spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //(polling for\setting) data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // Example 2: LPF definitions
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;

    // Example 3: HPF definitions
    // use with the example 2 definitions
    float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f;

    arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);

    int sample_count = 0;

    while (1) {
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        flags.wait_all(DATA_READY_FLAG);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        // Apply Simple low-pass filter
        filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

        // Apply simple high-pass filter
        high_pass_gx = gx - filtered_gx;
        high_pass_gy = gy - filtered_gy;
        high_pass_gz = gz - filtered_gz;

        // printf(">x_axis_high:%4.5f\n", high_pass_gx);
        // printf(">y_axis_high:%4.5f\n", high_pass_gy);
        // printf(">z_axis_high:%4.5f\n", high_pass_gz);

        fftIn_x[2 * sample_count] = high_pass_gx;
        fftIn_x[2 * sample_count + 1] = 0.0f;

        fftIn_y[2 * sample_count] = high_pass_gy;
        fftIn_y[2 * sample_count + 1] = 0.0f;

        fftIn_z[2 * sample_count] = high_pass_gz;
        fftIn_z[2 * sample_count + 1] = 0.0f;

        sample_count += 1;

        if (sample_count == SAMPLES){
            arm_rfft_fast_f32(&fftHandler, fftIn_x, fftOut_x, 0);
 
            for (int i = 0; i < FFT_BUFFER_SIZE / 2; i ++){
                float real = fftIn_x[2 * i];
                float img = fftIn_x[2 * i + 1];
                fftOut_x[i] = sqrtf(real * real + img * img);

                real = fftIn_y[2 * i];
                img = fftIn_y[2 * i + 1];
                fftOut_y[i] = sqrtf(real * real + img * img);

                real = fftIn_z[2 * i];
                img = fftIn_z[2 * i + 1];
                fftOut_z[i] = sqrtf(real * real + img * img);
            }

            int start = (3 * SAMPLES) / 100;
            int end = (6 * SAMPLES) / 100;

            float sum_x = 0;
            float sum_y = 0;
            float sum_z = 0;

            for (int i = start; i <= end; i++) {
                printf("\n");
                sum_x += fftOut_x[i];
                sum_y += fftOut_y[i];
                sum_z += fftOut_z[i];
            }

            float avg_x = sum_x / 3;
            float avg_y = sum_y / 3;
            float avg_z = sum_z / 3;

            printf("%f \n", avg_x);
            printf("%f \n", avg_y);
            printf("%f \n", avg_z);

            int tremor_detected = 0;
            if (avg_x >= 0.5 && avg_x < 2.5){
                tremor_detected += 1;
            }
            if (avg_y >= 0.5 && avg_y < 2.5){
                tremor_detected += 1;
            }
            if (avg_z >= 0.5 && avg_z < 2.5){
                tremor_detected += 1;
            }
            
            if (tremor_detected >= 2){
                printf("Tremor detected! \n");
            }

            // reset
            for (int i = 0; i < FFT_BUFFER_SIZE; i ++){
                fftIn_x[i] = 0;
                fftIn_y[i] = 0;
                fftIn_z[i] = 0;
                fftOut_x[i] = 0;
                fftOut_y[i] = 0;
                fftOut_z[i] = 0;
            }
            sample_count = 0;
        }
        thread_sleep_for(100);
    }
}