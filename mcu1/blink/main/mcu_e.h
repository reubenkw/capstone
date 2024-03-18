#ifndef MCU_E_H
#define MCU_E_H

void test_limit();
void test_all_stepper();
void test_z_stepper();
void test_i2c_stepper_interface();
void test_stepper_positioning();
void test_motor_go_to(int x, int y, int z);
void test_i2c_read_write();

#endif // h
