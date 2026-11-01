// Hand-written ALTPLL wrapper: 50 MHz in -> 18.4 MHz out (ratio 46/125)
// Use this INSTEAD of the MegaWizard-generated audio_pll (delete that one first).
// 18.4 MHz is fine: the WM8731 is the I2S master and derives its own timing.
module audio_pll (
    input  inclk0,
    output c0,
    output locked
);
    wire [4:0] clk_bus;
    wire       lock_w;

    altpll altpll_component (
        .inclk        ({1'b0, inclk0}),
        .clk          (clk_bus),
        .locked       (lock_w),
        .areset       (1'b0),
        .clkena       ({6{1'b1}}),
        .extclkena    ({4{1'b1}}),
        .fbin         (1'b1),
        .pfdena       (1'b1),
        .phasecounterselect ({4{1'b1}}),
        .phasestep    (1'b1),
        .phaseupdown  (1'b1),
        .pllena       (1'b1),
        .scanaclr     (1'b0),
        .scanclk      (1'b0),
        .scanclkena   (1'b1),
        .scandata     (1'b0),
        .scanread     (1'b0),
        .scanwrite    (1'b0),
        .clkswitch    (1'b0),
        .configupdate (1'b0)
    );

    defparam
        altpll_component.bandwidth_type         = "AUTO",
        altpll_component.clk0_divide_by         = 125,
        altpll_component.clk0_duty_cycle        = 50,
        altpll_component.clk0_multiply_by       = 46,
        altpll_component.clk0_phase_shift       = "0",
        altpll_component.compensate_clock       = "CLK0",
        altpll_component.inclk0_input_frequency = 20000,
        altpll_component.intended_device_family = "Cyclone IV E",
        altpll_component.lpm_type               = "altpll",
        altpll_component.operation_mode         = "NORMAL",
        altpll_component.pll_type               = "AUTO",
        altpll_component.port_areset            = "PORT_UNUSED",
        altpll_component.port_inclk0            = "PORT_USED",
        altpll_component.port_inclk1            = "PORT_UNUSED",
        altpll_component.port_locked            = "PORT_USED",
        altpll_component.port_clk0              = "PORT_USED",
        altpll_component.port_clk1              = "PORT_UNUSED",
        altpll_component.port_clk2              = "PORT_UNUSED",
        altpll_component.port_clk3              = "PORT_UNUSED",
        altpll_component.port_clk4              = "PORT_UNUSED",
        altpll_component.width_clock            = 5;

    assign c0     = clk_bus[0];
    assign locked = lock_w;
endmodule
