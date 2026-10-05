#include "unity.h"


/* =========================================================
 * Protocol Tests
 * ========================================================= */

void test_protocol_parser_accepts_valid_frame(void);
void test_protocol_parser_rejects_bad_checksum_and_recovers(void);
void test_protocol_parser_rejects_oversized_payload_and_recovers(void);
void test_protocol_parse_set_parameter(void);
void test_protocol_parse_set_parameter_rejects_invalid_length(void);
void test_protocol_build_parameter_ack(void);
void test_protocol_parse_diagnostics_default_selector(void);
void test_protocol_parse_joint_state_signed_values(void);


/* =========================================================
 * Motor Driver Tests
 * ========================================================= */

void test_motor_init_sets_zero_targets(void);
void test_motor_target_normalization(void);
void test_motor_first_feedback_has_zero_velocity(void);
void test_motor_wrap_forward(void);
void test_motor_wrap_backward(void);
void test_motor_null_pointer_errors(void);


/* =========================================================
 * Ring Buffer Tests
 * ========================================================= */

void test_ring_buffer_initially_empty(void);
void test_ring_buffer_preserves_fifo_order(void);
void test_ring_buffer_capacity_boundary(void);
void test_ring_buffer_wraparound(void);


/* =========================================================
 * UART Driver Tests
 * ========================================================= */

void test_uart_init_configures_hardware(void);
void test_uart_write_rejects_invalid_arguments(void);
void test_uart_tx_interrupt_drains_buffer(void);
void test_uart_tx_rejects_frame_larger_than_buffer(void);
void test_uart_rx_interrupt_receives_byte(void);
void test_uart_rx_overflow_increments_drop_count(void);


/* =========================================================
 * Unity Hooks
 * ========================================================= */

void setUp(void)
{
}


void tearDown(void)
{
}


/* =========================================================
 * Test Entry
 * ========================================================= */

int main(void)
{
    UNITY_BEGIN();


    /* =====================================================
     * Protocol
     * ===================================================== */

    RUN_TEST(
        test_protocol_parser_accepts_valid_frame
    );

    RUN_TEST(
        test_protocol_parser_rejects_bad_checksum_and_recovers
    );

    RUN_TEST(
        test_protocol_parser_rejects_oversized_payload_and_recovers
    );

    RUN_TEST(
        test_protocol_parse_set_parameter
    );

    RUN_TEST(
        test_protocol_parse_set_parameter_rejects_invalid_length
    );

    RUN_TEST(
        test_protocol_build_parameter_ack
    );

    RUN_TEST(
        test_protocol_parse_diagnostics_default_selector
    );

    RUN_TEST(
        test_protocol_parse_joint_state_signed_values
    );


    /* =====================================================
     * Motor Driver
     * ===================================================== */

    RUN_TEST(
        test_motor_init_sets_zero_targets
    );

    RUN_TEST(
        test_motor_target_normalization
    );

    RUN_TEST(
        test_motor_first_feedback_has_zero_velocity
    );

    RUN_TEST(
        test_motor_wrap_forward
    );

    RUN_TEST(
        test_motor_wrap_backward
    );

    RUN_TEST(
        test_motor_null_pointer_errors
    );


    /* =====================================================
     * Ring Buffer
     * ===================================================== */

    RUN_TEST(
        test_ring_buffer_initially_empty
    );

    RUN_TEST(
        test_ring_buffer_preserves_fifo_order
    );

    RUN_TEST(
        test_ring_buffer_capacity_boundary
    );

    RUN_TEST(
        test_ring_buffer_wraparound
    );


    /* =====================================================
     * UART Driver
     * ===================================================== */

    RUN_TEST(
        test_uart_init_configures_hardware
    );

    RUN_TEST(
        test_uart_write_rejects_invalid_arguments
    );

    RUN_TEST(
        test_uart_tx_interrupt_drains_buffer
    );

    RUN_TEST(
        test_uart_tx_rejects_frame_larger_than_buffer
    );

    RUN_TEST(
        test_uart_rx_interrupt_receives_byte
    );

    RUN_TEST(
        test_uart_rx_overflow_increments_drop_count
    );


    return UNITY_END();
}