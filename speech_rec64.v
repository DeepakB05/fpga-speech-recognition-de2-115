//=====================================================================
// 64-word speaker-dependent recognizer + VGA text display
// DE2-115 (Cyclone IV E).  Same audio front end as the 4-word version.
//
// Controls
//   KEY[3] reset (keeps trained words)   KEY[2] clear ALL trained words
//   KEY[0] TRAIN slot SW[5:0]            KEY[1] RECOGNIZE
//   SW[5:0]   slot number 0..63 (VGA shows the word to say for that slot)
//   SW[17:15] speech trigger threshold   SW[13:10] reject limit (x128, 0=off)
//   SW[14]=1 append recognized words to the on-screen sentence
//   SW[8] rising edge clears the sentence, SW[7]=1 hands-free listening
// Display
//   VGA: recognized WORD in big letters, selected slot + word to train
//   HEX1:HEX0 = recognized slot (decimal), HEX7..HEX4 = match distance (hex)
//   LEDR[0] = selected slot is trained, LEDG[0] = valid result,
//   LEDG[8] listening, LEDG[7] capturing, LEDR[15:8] mic level,
//   LEDR[16] sample heartbeat, LEDR[17] codec configured
// Word names: edit the WORDS table below (64 entries x exactly 8 chars).
//=====================================================================

module speech_top(
  input         CLOCK_50,
  input  [3:0]  KEY,
  input  [17:0] SW,
  output [17:0] LEDR,
  output [8:0]  LEDG,
  output [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
  output        I2C_SCLK,
  inout         I2C_SDAT,
  input         AUD_ADCDAT,
  input         AUD_ADCLRCK,
  input         AUD_BCLK,
  output        AUD_XCK,
  output        AUD_DACDAT,
  output reg [7:0] VGA_R,
  output reg [7:0] VGA_G,
  output reg [7:0] VGA_B,
  output        VGA_CLK,
  output reg    VGA_BLANK_N,
  output        VGA_SYNC_N,
  output reg    VGA_HS,
  output reg    VGA_VS
);
  wire clk = CLOCK_50;
  wire pll_locked;
  audio_pll u_pll (.inclk0(CLOCK_50), .c0(AUD_XCK), .locked(pll_locked));
  wire rst = ~KEY[3] | ~pll_locked;

  // ---------------- codec + capture + features (unchanged) ----------------
  wire cfg_done;
  i2c_wm8731_cfg u_cfg (.clk(clk), .rst(rst), .scl(I2C_SCLK), .sda(I2C_SDAT), .done(cfg_done));

  wire [15:0] smp;
  wire        smp_v;
  i2s_rx u_rx (.clk(clk), .bclk(AUD_BCLK), .lrck(AUD_ADCLRCK), .din(AUD_ADCDAT),
               .sample(smp), .valid(smp_v));
  assign AUD_DACDAT = 1'b0;

  reg         arm;
  wire        fe_done, fe_busy, fe_cap;
  wire [6:0]  level;
  wire [191:0] vec;
  wire [4:0]  thr_sh = 5'd16 + SW[17:15];

  feat_ext u_fe (.clk(clk), .rst(rst), .sample_valid(smp_v), .sample(smp),
                 .arm(arm), .thr_sh(thr_sh), .done(fe_done), .busy(fe_busy),
                 .capturing(fe_cap), .level(level), .vec(vec));

  // ---------------- keys ----------------
  reg [2:0] k0s, k1s, k2s;
  always @(posedge clk) begin
    k0s <= {k0s[1:0], KEY[0]};
    k1s <= {k1s[1:0], KEY[1]};
    k2s <= {k2s[1:0], KEY[2]};
  end
  wire k0_press = (k0s[2:1] == 2'b10);
  wire k1_press = (k1s[2:1] == 2'b10);
  wire k2_press = (k2s[2:1] == 2'b10);

  // ---------------- template RAM: 64 slots x 24 bytes ----------------
  reg  [5:0]  slot;
  reg  [5:0]  s;
  reg  [4:0]  e;
  reg  [2:0]  cs;        // 0 idle, 1 read, 2 accumulate, 3 write template, 4 decide
  reg  [7:0]  tmem [0:2047];
  reg  [7:0]  tq;
  always @(posedge clk) begin
    if (cs == 3'd3) tmem[{slot, e}] <= vec[e*8 +: 8];
    tq <= tmem[{s, e}];
  end

  // ---- hands-free listening: re-arm after ~0.33 s of idle ----
  reg [24:0] gap;
  wire gap_done = gap[24];
  always @(posedge clk) begin
    if (rst || fe_busy || cs != 3'd0 || arm) gap <= 25'd0;
    else if (!gap[24]) gap <= gap + 1'b1;
  end

  // ---------------- control / matching ----------------
  reg         train_mode;
  reg  [63:0] trained = 64'd0;
  reg  [13:0] dacc, bestd, mind;
  reg  [5:0]  besti, result;
  reg         res_valid;
  wire [5:0]  sel = SW[5:0];

  wire signed [7:0] ve = vec[e*8 +: 8];
  wire signed [7:0] te = tq;
  wire signed [8:0] df = ve - te;
  wire [8:0]  ad    = df[8] ? -df : df;
  wire [13:0] dnext = dacc + ad;
  wire [13:0] rej_thr = {SW[13:10], 7'b0};

  always @(posedge clk) begin
    arm <= 1'b0;
    if (rst) begin
      train_mode <= 1'b0; slot <= 6'd0; cs <= 3'd0; s <= 6'd0; e <= 5'd0;
      dacc <= 14'd0; bestd <= 14'h3FFF; besti <= 6'd0; result <= 6'd0;
      res_valid <= 1'b0; mind <= 14'h3FFF;
    end else begin
      if (k2_press) begin trained <= 64'd0; res_valid <= 1'b0; end

      if (cs == 3'd0 && !fe_busy && !arm && cfg_done) begin
        if (k0_press) begin
          train_mode <= 1'b1; slot <= sel; arm <= 1'b1; res_valid <= 1'b0;
        end else if (k1_press) begin
          train_mode <= 1'b0; arm <= 1'b1; res_valid <= 1'b0;
        end else if (SW[7] && !train_mode && gap_done) begin
          arm <= 1'b1;                       // hands-free: listen for the next word
        end
      end

      if (fe_done) begin
        if (train_mode) begin
          cs <= 3'd3; e <= 5'd0;
        end else begin
          cs <= 3'd1; s <= 6'd0; e <= 5'd0; dacc <= 14'd0;
          bestd <= 14'h3FFF; besti <= 6'd0;
        end
      end

      case (cs)
        3'd1: cs <= 3'd2;                              // wait for RAM read
        3'd2: begin
          if (e == 5'd23) begin
            if (trained[s] && (dnext < bestd)) begin
              bestd <= dnext; besti <= s;
            end
            dacc <= 14'd0; e <= 5'd0;
            if (s == 6'd63) cs <= 3'd4;
            else begin s <= s + 1'b1; cs <= 3'd1; end
          end else begin
            dacc <= dnext; e <= e + 1'b1; cs <= 3'd1;
          end
        end
        3'd3: begin                                    // write 24 template bytes
          if (e == 5'd23) begin
            trained[slot] <= 1'b1; cs <= 3'd0; e <= 5'd0;
          end else e <= e + 1'b1;
        end
        3'd4: begin
          result    <= besti;
          mind      <= bestd;
          res_valid <= (bestd != 14'h3FFF) && ((SW[13:10] == 4'd0) || (bestd <= rej_thr));
          cs        <= 3'd0;
        end
        default: ;
      endcase
    end
  end

  // ---------------- 7-seg, LEDs ----------------
  function [6:0] hex7;
    input [3:0] d;
    begin
      case (d)
        4'h0: hex7 = 7'b1000000; 4'h1: hex7 = 7'b1111001;
        4'h2: hex7 = 7'b0100100; 4'h3: hex7 = 7'b0110000;
        4'h4: hex7 = 7'b0011001; 4'h5: hex7 = 7'b0010010;
        4'h6: hex7 = 7'b0000010; 4'h7: hex7 = 7'b1111000;
        4'h8: hex7 = 7'b0000000; 4'h9: hex7 = 7'b0010000;
        4'hA: hex7 = 7'b0001000; 4'hB: hex7 = 7'b0000011;
        4'hC: hex7 = 7'b1000110; 4'hD: hex7 = 7'b0100001;
        4'hE: hex7 = 7'b0000110; default: hex7 = 7'b0001110;
      endcase
    end
  endfunction

  function [3:0] tens;
    input [5:0] v;
    begin
      if (v >= 6'd60) tens = 4'd6;
      else if (v >= 6'd50) tens = 4'd5;
      else if (v >= 6'd40) tens = 4'd4;
      else if (v >= 6'd30) tens = 4'd3;
      else if (v >= 6'd20) tens = 4'd2;
      else if (v >= 6'd10) tens = 4'd1;
      else tens = 4'd0;
    end
  endfunction

  wire [3:0] res_t = tens(result);
  wire [3:0] res_u = result - res_t * 10;
  wire [3:0] sel_t = tens(sel);
  wire [3:0] sel_u = sel - sel_t * 10;

  wire [15:0] mind16 = {2'b00, mind};
  assign HEX0 = res_valid ? hex7(res_u) : 7'b0111111;
  assign HEX1 = res_valid ? hex7(res_t) : 7'b0111111;
  assign HEX2 = 7'h7F;
  assign HEX3 = 7'h7F;
  assign HEX4 = hex7(mind16[3:0]);
  assign HEX5 = hex7(mind16[7:4]);
  assign HEX6 = hex7(mind16[11:8]);
  assign HEX7 = hex7(mind16[15:12]);

  assign LEDG = {fe_busy, fe_cap, 6'b000000, res_valid};

  reg [12:0] hbc;
  always @(posedge clk) if (smp_v) hbc <= hbc + 1'b1;

  genvar g;
  generate
    for (g = 0; g < 8; g = g + 1) begin : bar
      assign LEDR[8+g] = (level >= (7'd48 + g*8));
    end
  endgenerate
  assign LEDR[17]  = cfg_done;
  assign LEDR[16]  = hbc[12];
  assign LEDR[7:1] = 7'b0000000;
  assign LEDR[0]   = trained[sel];

  //===================================================================
  // Word table: 64 entries, EXACTLY 8 characters each (pad with spaces).
  // Allowed characters: A-Z 0-9 '-' and space. Entry 0 is the first line.
  //===================================================================
  localparam [8*512-1:0] WORDS = {
    "ZERO    ", "ONE     ", "TWO     ", "THREE   ",
    "FOUR    ", "FIVE    ", "SIX     ", "SEVEN   ",
    "EIGHT   ", "NINE    ", "TEN     ", "YES     ",
    "NO      ", "STOP    ", "GO      ", "START   ",
    "LEFT    ", "RIGHT   ", "UP      ", "DOWN    ",
    "ON      ", "OFF     ", "BACK    ", "FRONT   ",
    "OPEN    ", "CLOSE   ", "HELLO   ", "BYE     ",
    "HELP    ", "PLAY    ", "PAUSE   ", "NEXT    ",
    "RED     ", "GREEN   ", "BLUE    ", "WHITE   ",
    "BLACK   ", "YELLOW  ", "ORANGE  ", "PINK    ",
    "DOG     ", "CAT     ", "BIRD    ", "FISH    ",
    "TREE    ", "WATER   ", "FIRE    ", "EARTH   ",
    "SUN     ", "MOON    ", "STAR    ", "RAIN    ",
    "SNOW    ", "WIND    ", "DAY     ", "NIGHT   ",
    "MORNING ", "HOUSE   ", "CAR     ", "BOOK    ",
    "MY      ", "NAME    ", "IS      ", "DEEPAK  "
  };

  //===================================================================
  // Sentence builder: every recognized word is appended to a 3-line x
  // 20-character text area at the bottom of the screen (word-wrapped).
  //   SW[14]=1       : append each valid recognized word
  //   SW[8] rising   : clear the sentence
  //   SW[7]=1        : hands-free (re-arms itself after each word)
  //===================================================================
  wire valid_now = (bestd != 14'h3FFF) && ((SW[13:10] == 4'd0) || (bestd <= rej_thr));
  wire new_word  = (cs == 3'd4) && valid_now && SW[14];

  (* ramstyle = "logic" *) reg [7:0] sbuf [0:63];
  reg [2:0] sst;                    // 0 idle, 1 clear, 2 check fit, 3 write word, 4 add space
  reg [6:0] cptr;
  reg [1:0] wline;
  reg [4:0] wcol;
  reg [5:0] wslot;
  reg [2:0] wj;
  reg       redo;
  reg [2:0] sw8s;
  always @(posedge clk) sw8s <= {sw8s[1:0], SW[8]};
  wire clr_req = (sw8s[2:1] == 2'b01);

  wire [63:0] wbits = WORDS[(6'd63 - wslot) * 64 +: 64];
  wire [7:0]  wchar = wbits[(7 - wj) * 8 +: 8];
  wire [3:0]  wlen  = (wbits[63:56] != 8'h20) + (wbits[55:48] != 8'h20) +
                      (wbits[47:40] != 8'h20) + (wbits[39:32] != 8'h20) +
                      (wbits[31:24] != 8'h20) + (wbits[23:16] != 8'h20) +
                      (wbits[15:8]  != 8'h20) + (wbits[7:0]   != 8'h20);
  wire [6:0]  wadr  = wline * 7'd20 + wcol;

  always @(posedge clk) begin
    if (rst) begin
      sst <= 3'd1; cptr <= 7'd0; redo <= 1'b0; wline <= 2'd0; wcol <= 5'd0; wj <= 3'd0;
    end else begin
      case (sst)
        3'd0: begin
          if (clr_req) begin
            sst <= 3'd1; cptr <= 7'd0; redo <= 1'b0;
          end else if (new_word) begin
            wslot <= besti; sst <= 3'd2;
          end
        end
        3'd1: begin                                   // clear whole buffer
          sbuf[cptr[5:0]] <= 8'h20;
          cptr <= cptr + 1'b1;
          if (cptr == 7'd63) begin
            wline <= 2'd0; wcol <= 5'd0; redo <= 1'b0;
            sst <= redo ? 3'd2 : 3'd0;
          end
        end
        3'd2: begin                                   // does the word fit on this line?
          if (wlen == 4'd0) sst <= 3'd0;
          else if (wline == 2'd3) begin
            sst <= 3'd1; cptr <= 7'd0; redo <= 1'b1;  // screen full: start over
          end else if (wcol + wlen > 5'd20) begin
            if (wline == 2'd2) begin
              sst <= 3'd1; cptr <= 7'd0; redo <= 1'b1;
            end else begin
              wline <= wline + 1'b1; wcol <= 5'd0; wj <= 3'd0; sst <= 3'd3;
            end
          end else begin
            wj <= 3'd0; sst <= 3'd3;
          end
        end
        3'd3: begin                                   // copy the 8 characters, skip padding
          if (wchar != 8'h20) begin
            sbuf[wadr[5:0]] <= wchar;
            wcol <= wcol + 1'b1;
          end
          wj <= wj + 1'b1;
          if (wj == 3'd7) sst <= 3'd4;
        end
        3'd4: begin                                   // one space after the word
          if (wcol >= 5'd19) begin wcol <= 5'd0; wline <= wline + 1'b1; end
          else wcol <= wcol + 1'b1;
          sst <= 3'd0;
        end
        default: sst <= 3'd0;
      endcase
    end
  end

  //===================================================================
  // VGA 640x480 @ ~60 Hz, 25 MHz pixel rate from the 50 MHz clock
  //===================================================================
  reg pix;
  always @(posedge clk) pix <= ~pix;
  wire tick = ~pix;                 // pixel pipeline advances on these edges
  assign VGA_CLK    = ~pix;         // rising edge lands mid-way through each pixel's data
  assign VGA_SYNC_N = 1'b0;

  reg [9:0] hcnt, vcnt;
  always @(posedge clk) if (tick) begin
    if (hcnt == 10'd799) begin
      hcnt <= 10'd0;
      vcnt <= (vcnt == 10'd524) ? 10'd0 : vcnt + 1'b1;
    end else hcnt <= hcnt + 1'b1;
  end

  // ---- stage 1: which text row / character cell / glyph pixel ----
  reg [2:0] r_row;
  reg       r_in;
  reg [9:0] r_y0, r_x0;
  reg [1:0] r_sh;
  reg [4:0] r_nc;
  always @* begin
    r_in = 1'b1; r_row = 3'd0; r_y0 = 10'd0; r_x0 = 10'd0; r_sh = 2'd2; r_nc = 5'd0;
    if      (vcnt >= 10'd16  && vcnt < 10'd48 ) begin r_row=3'd0; r_y0=10'd16;  r_x0=10'd48;  r_nc=5'd17; end
    else if (vcnt >= 10'd80  && vcnt < 10'd112) begin r_row=3'd1; r_y0=10'd80;  r_x0=10'd112; r_nc=5'd13; end
    else if (vcnt >= 10'd120 && vcnt < 10'd152) begin r_row=3'd2; r_y0=10'd120; r_x0=10'd128; r_nc=5'd12; end
    else if (vcnt >= 10'd200 && vcnt < 10'd232) begin r_row=3'd3; r_y0=10'd200; r_x0=10'd240; r_nc=5'd5;  end
    else if (vcnt >= 10'd240 && vcnt < 10'd304) begin r_row=3'd4; r_y0=10'd240; r_x0=10'd64;  r_nc=5'd8;  r_sh=2'd3; end
    else if (vcnt >= 10'd340 && vcnt < 10'd372) begin r_row=3'd5; r_y0=10'd340; r_x0=10'd112; r_nc=5'd13; end
    else if (vcnt >= 10'd384 && vcnt < 10'd480) begin r_row=3'd6; r_y0=10'd384; r_x0=10'd0;   r_nc=5'd20; end
    else r_in = 1'b0;
  end
  wire [9:0] rel_y = vcnt - r_y0;
  wire [9:0] rel_x = hcnt - r_x0;
  wire [2:0] gy_w  = rel_y >> r_sh;
  wire [2:0] gx_w  = rel_x >> r_sh;
  wire [2:0] csh   = {1'b0, r_sh} + 3'd3;   // 5 or 6: needs 3 bits
  wire [4:0] col_w = rel_x >> csh;
  wire       xin_w = (hcnt >= r_x0) && (col_w < r_nc);

  reg       s1_in, s1_act, s1_hs, s1_vs;
  reg [1:0] s1_line;
  reg [2:0] s1_row, s1_gx, s1_gy;
  reg [4:0] s1_col;
  always @(posedge clk) if (tick) begin
    s1_in  <= r_in & xin_w;
    s1_row <= r_row;
    s1_line <= rel_y[6:5];
    s1_col <= col_w;
    s1_gx  <= gx_w;
    s1_gy  <= gy_w;
    s1_act <= (hcnt < 10'd640) && (vcnt < 10'd480);
    s1_hs  <= ~((hcnt >= 10'd656) && (hcnt < 10'd752));
    s1_vs  <= ~((vcnt >= 10'd490) && (vcnt < 10'd492));
  end

  // ---- stage 2: character code for this cell ----
  localparam [8*17-1:0] S_A = "SPEECH RECOGNIZER";
  localparam [8*11-1:0] S_B = "TRAIN SLOT ";
  localparam [8*4-1:0]  S_C = "SAY ";
  localparam [8*5-1:0]  S_D = "HEARD";
  localparam [8*9-1:0]  S_L = "LISTENING";
  localparam [8*11-1:0] S_M = "MATCH SLOT ";
  localparam [8*5-1:0]  S_R = "READY";

  wire [5:0] lslot = (s1_row == 3'd2) ? sel : result;
  wire [2:0] lci   = (s1_row == 3'd2) ? (s1_col[2:0] - 3'd4) : s1_col[2:0];
  wire [8:0] lwi   = {lslot, lci};
  wire [7:0] lasc  = WORDS[(9'd511 - lwi) * 8 +: 8];
  wire [6:0] saddr = s1_line * 7'd20 + s1_col;

  reg [7:0] ta;
  always @* begin
    ta = 8'h20;
    case (s1_row)
      3'd0: ta = S_A[(16 - s1_col) * 8 +: 8];
      3'd1: begin
        if (s1_col < 5'd11)       ta = S_B[(10 - s1_col) * 8 +: 8];
        else if (s1_col == 5'd11) ta = 8'd48 + sel_t;
        else                      ta = 8'd48 + sel_u;
      end
      3'd2: begin
        if (s1_col < 5'd4) ta = S_C[(3 - s1_col) * 8 +: 8];
        else               ta = lasc;
      end
      3'd3: ta = S_D[(4 - s1_col) * 8 +: 8];
      3'd4: ta = res_valid ? lasc : 8'h2D;               // '-' when no result
      3'd5: begin
        if (fe_busy) begin
          if (s1_col < 5'd9) ta = S_L[(8 - s1_col) * 8 +: 8];
        end else if (res_valid) begin
          if (s1_col < 5'd11)       ta = S_M[(10 - s1_col) * 8 +: 8];
          else if (s1_col == 5'd11) ta = 8'd48 + res_t;
          else                      ta = 8'd48 + res_u;
        end else begin
          if (s1_col < 5'd5) ta = S_R[(4 - s1_col) * 8 +: 8];
        end
      end
      3'd6: ta = sbuf[saddr[5:0]];                      // sentence text area
      default: ta = 8'h20;
    endcase
  end

  function [5:0] a2c;
    input [7:0] a;
    begin
      if (a >= "A" && a <= "Z")      a2c = a - "A" + 6'd1;
      else if (a >= "0" && a <= "9") a2c = a - "0" + 6'd27;
      else if (a == "-")             a2c = 6'd37;
      else                           a2c = 6'd0;
    end
  endfunction

  reg       s2_act, s2_hs, s2_vs;
  reg [2:0] s2_row, s2_gx, s2_gy;
  reg [5:0] s2_ch;
  always @(posedge clk) if (tick) begin
    s2_ch  <= s1_in ? a2c(ta) : 6'd0;
    s2_row <= s1_row;
    s2_gx  <= s1_gx;
    s2_gy  <= s1_gy;
    s2_act <= s1_act;
    s2_hs  <= s1_hs;
    s2_vs  <= s1_vs;
  end

  // ---- stage 3: 5x7 font -> pixel colour ----
  reg [34:0] gl;
  always @* begin
    case (s2_ch)
      6'd1:  gl = 35'b01110_10001_10001_11111_10001_10001_10001; // A
      6'd2:  gl = 35'b11110_10001_10001_11110_10001_10001_11110; // B
      6'd3:  gl = 35'b01110_10001_10000_10000_10000_10001_01110; // C
      6'd4:  gl = 35'b11110_10001_10001_10001_10001_10001_11110; // D
      6'd5:  gl = 35'b11111_10000_10000_11110_10000_10000_11111; // E
      6'd6:  gl = 35'b11111_10000_10000_11110_10000_10000_10000; // F
      6'd7:  gl = 35'b01110_10001_10000_10111_10001_10001_01111; // G
      6'd8:  gl = 35'b10001_10001_10001_11111_10001_10001_10001; // H
      6'd9:  gl = 35'b01110_00100_00100_00100_00100_00100_01110; // I
      6'd10: gl = 35'b00111_00010_00010_00010_00010_10010_01100; // J
      6'd11: gl = 35'b10001_10010_10100_11000_10100_10010_10001; // K
      6'd12: gl = 35'b10000_10000_10000_10000_10000_10000_11111; // L
      6'd13: gl = 35'b10001_11011_10101_10101_10001_10001_10001; // M
      6'd14: gl = 35'b10001_11001_10101_10011_10001_10001_10001; // N
      6'd15: gl = 35'b01110_10001_10001_10001_10001_10001_01110; // O
      6'd16: gl = 35'b11110_10001_10001_11110_10000_10000_10000; // P
      6'd17: gl = 35'b01110_10001_10001_10001_10101_10010_01101; // Q
      6'd18: gl = 35'b11110_10001_10001_11110_10100_10010_10001; // R
      6'd19: gl = 35'b01111_10000_10000_01110_00001_00001_11110; // S
      6'd20: gl = 35'b11111_00100_00100_00100_00100_00100_00100; // T
      6'd21: gl = 35'b10001_10001_10001_10001_10001_10001_01110; // U
      6'd22: gl = 35'b10001_10001_10001_10001_10001_01010_00100; // V
      6'd23: gl = 35'b10001_10001_10001_10101_10101_10101_01010; // W
      6'd24: gl = 35'b10001_10001_01010_00100_01010_10001_10001; // X
      6'd25: gl = 35'b10001_10001_01010_00100_00100_00100_00100; // Y
      6'd26: gl = 35'b11111_00001_00010_00100_01000_10000_11111; // Z
      6'd27: gl = 35'b01110_10001_10011_10101_11001_10001_01110; // 0
      6'd28: gl = 35'b00100_01100_00100_00100_00100_00100_01110; // 1
      6'd29: gl = 35'b01110_10001_00001_00010_00100_01000_11111; // 2
      6'd30: gl = 35'b11110_00001_00001_01110_00001_00001_11110; // 3
      6'd31: gl = 35'b00010_00110_01010_10010_11111_00010_00010; // 4
      6'd32: gl = 35'b11111_10000_11110_00001_00001_10001_01110; // 5
      6'd33: gl = 35'b00110_01000_10000_11110_10001_10001_01110; // 6
      6'd34: gl = 35'b11111_00001_00010_00100_01000_01000_01000; // 7
      6'd35: gl = 35'b01110_10001_10001_01110_10001_10001_01110; // 8
      6'd36: gl = 35'b01110_10001_10001_01111_00001_00010_01100; // 9
      6'd37: gl = 35'b00000_00000_00000_11111_00000_00000_00000; // -
      default: gl = 35'd0;                                         // space
    endcase
  end

  wire [4:0] frow = (s2_gy < 3'd7) ? gl[(34 - 5 * s2_gy) -: 5] : 5'b00000;
  wire       pxon = (s2_gx < 3'd5) && frow[4 - s2_gx];

  reg [23:0] fg;
  always @* begin
    case (s2_row)
      3'd1, 3'd2: fg = 24'hFFDC00;     // yellow
      3'd3:       fg = 24'h00DCFF;     // cyan
      3'd4:       fg = 24'h3CFF3C;     // green (the recognized word)
      3'd6:       fg = 24'hFFA040;     // orange (the sentence)
      default:    fg = 24'hFFFFFF;     // white
    endcase
  end

  always @(posedge clk) if (tick) begin
    {VGA_R, VGA_G, VGA_B} <= s2_act ? (pxon ? fg : 24'h000030) : 24'h000000;
    VGA_BLANK_N <= s2_act;
    VGA_HS      <= s2_hs;
    VGA_VS      <= s2_vs;
  end
endmodule


//=====================================================================
// I2S receiver (codec is master). Takes LEFT channel, 16 bit.
//=====================================================================
module i2s_rx(
  input             clk,
  input             bclk,
  input             lrck,
  input             din,
  output reg [15:0] sample,
  output reg        valid
);
  reg [2:0] b, l;
  reg       d1, d2;
  always @(posedge clk) begin
    b  <= {b[1:0], bclk};
    l  <= {l[1:0], lrck};
    d1 <= din;
    d2 <= d1;
  end
  wire brise = (b[2:1] == 2'b01);
  wire lfall = (l[2:1] == 2'b10);   // start of LEFT word

  reg [4:0]  cnt;
  reg        act;
  reg [15:0] sh;
  always @(posedge clk) begin
    valid <= 1'b0;
    if (lfall) begin
      act <= 1'b1;
      cnt <= 5'd0;
    end else if (brise && act) begin
      cnt <= cnt + 1'b1;
      if (cnt != 5'd0) sh <= {sh[14:0], d2};   // cnt==0: 1-bit I2S delay
      if (cnt == 5'd16) begin
        sample <= {sh[14:0], d2};
        valid  <= 1'b1;
        act    <= 1'b0;
      end
    end
  end
endmodule


//=====================================================================
// WM8731 configuration over I2C (address 0x34, ~50 kHz), write-only
//=====================================================================
module i2c_wm8731_cfg(
  input      clk,
  input      rst,
  output     scl,
  inout      sda,
  output reg done
);
  localparam N = 7;
  reg [20:0] pu;
  reg [7:0]  tcnt;
  reg [1:0]  ph;
  reg [4:0]  slot;
  reg [7:0]  gap;
  reg [3:0]  idx;

  wire run  = pu[20];
  wire t249 = (tcnt == 8'd249);
  wire tick = run & t249;

  reg [15:0] word;
  always @* begin
    case (idx)
      4'd0:    word = {7'd15, 9'h000};  // reset
      4'd1:    word = {7'd6,  9'h000};  // power up everything
      4'd2:    word = {7'd4,  9'h005};  // analog path: MIC in, boost on, mic unmuted
      4'd3:    word = {7'd5,  9'h000};  // digital path: ADC HPF on
      4'd4:    word = {7'd7,  9'h042};  // I2S, 16 bit, codec master
      4'd5:    word = {7'd8,  9'h002};  // normal mode, 384fs, 48 kHz
      default: word = {7'd9,  9'h001};  // activate
    endcase
  end
  wire [26:0] fr = {8'h34, 1'b1, word[15:8], 1'b1, word[7:0], 1'b1};

  reg scl_r, sda_low;
  always @* begin
    scl_r = 1'b1; sda_low = 1'b0;
    if (slot == 5'd0) begin                       // START
      case (ph)
        2'd0: ;
        2'd1, 2'd2: sda_low = 1'b1;
        default: begin scl_r = 1'b0; sda_low = 1'b1; end
      endcase
    end else if (slot <= 5'd27) begin             // 24 data bits + 3 ack slots
      sda_low = ~fr[27 - slot];
      scl_r   = (ph == 2'd1) || (ph == 2'd2);
    end else if (slot == 5'd28) begin             // STOP
      case (ph)
        2'd0: begin scl_r = 1'b0; sda_low = 1'b1; end
        2'd1: sda_low = 1'b1;
        default: sda_low = 1'b0;
      endcase
    end
  end
  assign scl = scl_r;
  assign sda = sda_low ? 1'b0 : 1'bz;

  always @(posedge clk) begin
    if (rst) begin
      pu <= 21'd0; tcnt <= 8'd0; ph <= 2'd0; slot <= 5'd0;
      gap <= 8'd0; idx <= 4'd0; done <= 1'b0;
    end else begin
      if (!run) pu <= pu + 1'b1;
      tcnt <= t249 ? 8'd0 : tcnt + 1'b1;
      if (tick && !done) begin
        ph <= ph + 1'b1;
        if (ph == 2'd3) begin
          if (slot == 5'd29) begin                // inter-transaction gap
            gap <= gap + 1'b1;
            if (gap == 8'd255) begin
              slot <= 5'd0;
              idx  <= idx + 1'b1;
              if (idx == N-1) done <= 1'b1;
            end
          end else slot <= slot + 1'b1;
        end
      end
    end
  end
endmodule


//=====================================================================
// Feature extractor: energy trigger, 64 frames, 8 segments x 3 features
//=====================================================================
module feat_ext(
  input               clk,
  input               rst,
  input               sample_valid,
  input signed [15:0] sample,
  input               arm,
  input        [4:0]  thr_sh,
  output reg          done,
  output              busy,
  output              capturing,
  output reg   [6:0]  level,
  output reg  [191:0] vec
);
  // ---- per-frame accumulation (512 samples) ----
  reg signed [15:0] prev;
  reg [25:0] accE, accH, fE, fH;
  reg [9:0]  zc, fZ;
  reg [8:0]  fcnt;
  reg        frame_rdy;

  wire [15:0] absx = sample[15] ? (~sample + 1'b1) : sample;
  wire signed [16:0] dif = {sample[15], sample} - {prev[15], prev};
  wire [16:0] absd = dif[16] ? (~dif + 1'b1) : dif;
  wire        zcx  = sample[15] ^ prev[15];
  wire [25:0] accE_n = accE + absx;
  wire [25:0] accH_n = accH + absd;
  wire [9:0]  zc_n   = zc + zcx;

  always @(posedge clk) begin
    frame_rdy <= 1'b0;
    if (rst) begin
      accE <= 0; accH <= 0; zc <= 0; fcnt <= 0; prev <= 0;
    end else if (sample_valid) begin
      prev <= sample;
      if (fcnt == 9'd511) begin
        fE <= accE_n; fH <= accH_n; fZ <= zc_n; frame_rdy <= 1'b1;
        accE <= 0; accH <= 0; zc <= 0; fcnt <= 0;
      end else begin
        accE <= accE_n; accH <= accH_n; zc <= zc_n; fcnt <= fcnt + 1'b1;
      end
    end
  end

  // ---- cheap log: {msb position, next 2 bits} ----
  function [6:0] lg;
    input [25:0] v;
    integer k;
    reg [4:0] p;
    reg [1:0] m;
    begin
      p = 5'd0;
      for (k = 0; k < 26; k = k + 1) if (v[k]) p = k[4:0];
      if (p >= 5'd2)      m = (v >> (p - 5'd2));
      else if (p == 5'd1) m = {v[0], 1'b0};
      else                m = 2'b00;
      lg = {p, m};
    end
  endfunction

  wire [6:0] lE = lg(fE);
  wire [6:0] lH = lg(fH);
  wire signed [7:0] f1 = {1'b0, lE};                                   // log energy
  wire signed [7:0] f2 = $signed({1'b0, lH}) - $signed({1'b0, lE});    // HF/LF tilt
  wire signed [7:0] f3 = {1'b0, fZ[9:3]};                              // zero crossings /8

  always @(posedge clk) if (frame_rdy) level <= lE;

  // ---- segment accumulators ----
  reg signed [11:0] a1 [0:7];
  reg signed [11:0] a2 [0:7];
  reg signed [11:0] a3 [0:7];
  reg signed [11:0] mean1;
  wire signed [13:0] sum1 = a1[0]+a1[1]+a1[2]+a1[3]+a1[4]+a1[5]+a1[6]+a1[7];

  localparam S_OFF = 3'd0, S_WAIT = 3'd1, S_CAP = 3'd2, S_N1 = 3'd3, S_N2 = 3'd4;
  reg [2:0] st;
  reg [5:0] fidx;
  integer i;
  wire [2:0]  seg = fidx[5:3];
  wire [25:0] thr = 26'd1 << thr_sh;

  assign busy      = (st != S_OFF);
  assign capturing = (st == S_CAP);

  always @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
      st <= S_OFF; fidx <= 6'd0;
    end else begin
      case (st)
        S_OFF: if (arm) st <= S_WAIT;

        S_WAIT: begin
          for (i = 0; i < 8; i = i + 1) begin
            a1[i] <= 12'sd0; a2[i] <= 12'sd0; a3[i] <= 12'sd0;
          end
          if (frame_rdy && (fE > thr)) begin     // speech onset: this frame is frame 0
            a1[0] <= f1; a2[0] <= f2; a3[0] <= f3;
            fidx  <= 6'd1;
            st    <= S_CAP;
          end
        end

        S_CAP: if (frame_rdy) begin
          a1[seg] <= a1[seg] + f1;
          a2[seg] <= a2[seg] + f2;
          a3[seg] <= a3[seg] + f3;
          fidx    <= fidx + 1'b1;
          if (fidx == 6'd63) st <= S_N1;
        end

        S_N1: begin
          mean1 <= sum1 >>> 6;                   // mean log-energy of utterance
          st    <= S_N2;
        end

        S_N2: begin
          for (i = 0; i < 8; i = i + 1) begin
            vec[(i*3)*8   +: 8] <= (a1[i] >>> 3) - mean1;   // loudness-normalised energy
            vec[(i*3+1)*8 +: 8] <= (a2[i] >>> 3);
            vec[(i*3+2)*8 +: 8] <= (a3[i] >>> 3);
          end
          done <= 1'b1;
          st   <= S_OFF;
        end

        default: st <= S_OFF;
      endcase
    end
  end
endmodule
