// CoProZ80.v
//
// Acorn Z80 Second Processor (parasite) for the MiST BBC Micro core.
//
// MiST adaptation of the developer's original CoProZ80.vhd (hoglet67).
// It applies exactly the same transformation that produced CoPro6502.v
// (the MiST 6502 co-pro) from CoPro6502.vhd, and instantiates the MiST
// split-bus Tube core (tube.v / tube_comp_pack):
//
//   * Host side uses the split data bus h_data_in / h_data_out and the host
//     clock h_clk (driving Tube h_phi2), replacing the board's inout h_data.
//   * The parasite clock (clk_cpu) and clock enable (cpu_clken) are supplied
//     by the MiST top level, replacing the on-board dcm_49_24 + clk_gen.
//   * The Tube parasite side runs at clk_cpu (p_phi2 = 48 MHz). The Tube
//     select is a ONE-CYCLE strobe fired on the assertion edge of the I/O
//     decode (one FIFO operation per Z80 access, like the real ULA), and the
//     Tube read data is captured at the strobe and HELD for the rest of the
//     access (valid wherever the T80 samples DI, like the real ULA). Without
//     this, the 48 MHz p_phi2 lets the gen_flag_v3 machines repeat operations
//     within one access (the real boards were protected by the ~1 us req/ack
//     round trip through the 2 MHz host bus), and a narrow select alone
//     breaks reads because hp_bytequad's data mux is select-qualified.
//     See the tube_strobe block below for the full analysis.
//   * External RAM uses the split ram_data_in / ram_data_out interface (64K).
//   * The Z80 boot ROM is acorn_z80_rom (rom_sel = 0 -> v1.20, 1 -> v1.21).
//
// The Z80-specific behaviour from the original CoProZ80.vhd is preserved:
// I/O-space Tube decode (&00-&07), boot-ROM paging via the &0066 NMI handler /
// A15 logic, the interrupt-acknowledge 0xFE (RST 38) vector, and p_rdnw =
// cpu_wr_n (read/write direction by strobe).

module CoProZ80(
	// Host
	input         h_clk,
	input         h_cs_b,
	input         h_rdnw,
	input   [2:0] h_addr,
	input   [7:0] h_data_in,
	output  [7:0] h_data_out,
	input         h_rst_b,
	output        h_irq_b,

	// Parasite clock (48 MHz) and 6 MHz clock enable
	input         clk_cpu,
	input         cpu_clken,

	// Boot ROM select (0 = v1.20, 1 = v1.21)
	input         rom_sel,

	// External RAM (64K)
	output [15:0] ram_addr,
	output  [7:0] ram_data_in,
	input   [7:0] ram_data_out,
	output        ram_wr,

	// Test signals for debugging
	output  [7:0] test
);

//-----------------------------------------------
// clock and reset signals
//-----------------------------------------------

	reg        bootmode;
	wire       RSTn;
	reg        RSTn_sync;
	reg  [8:0] reset_counter;
	reg  [3:0] clken_counter;

//-----------------------------------------------
// parasite signals (Tube)
//-----------------------------------------------

	wire       p_cs_b;       // raw Tube chip select (I/O decode, spans the whole I/O cycle)
	reg        p_cs_b_q;     // p_cs_b registered at clk_cpu (assertion-edge detector)
	wire       tube_cs_b;    // one-cycle Tube select strobe (exactly one per access)
	reg  [7:0] p_data_hold;  // Tube read data captured at the strobe, held for the access
	reg        p_hold_valid;
	wire [7:0] p_data_cpu;   // what the CPU actually sees: live during the strobe, held after
	wire [7:0] p_data_out;

//-----------------------------------------------
// ram/rom signals
//-----------------------------------------------

	wire       ram_cs_b;
	wire       rom_cs_b;
	wire [7:0] rom_data_out;

//-----------------------------------------------
// cpu signals
//-----------------------------------------------

	wire        cpu_rd_n;
	wire        cpu_wr_n;
	wire        cpu_iorq_n;
	wire        cpu_mreq_n;
	wire        cpu_m1_n;
	wire [15:0] cpu_addr;
	wire  [7:0] cpu_din;
	wire  [7:0] cpu_dout;
	wire        cpu_IRQ_n;
	wire        cpu_NMI_n;
	reg         cpu_IRQ_n_sync;
	reg         cpu_NMI_n_sync;

//-------------------------------------------------------------------
// instantiated components
//-------------------------------------------------------------------

	// Boot ROM (dual, OSD selectable). Repeats every 4K via the low address bits.
	acorn_z80_rom inst_tuberom (
		.CLK     ( clk_cpu ),
		.ADDR    ( cpu_addr[11:0] ),
		.ROM_SEL ( rom_sel ),
		.DATA    ( rom_data_out )
	);

	// Z80 CPU (T80se, standard I/O wait state)
	T80se inst_Z80 (
		.RESET_n ( RSTn_sync ),
		.CLK_n   ( clk_cpu ),         // 48 MHz
		.CLKEN   ( cpu_clken ),       //  6 MHz clock enable
		.WAIT_n  ( 1'b1 ),
		.INT_n   ( cpu_IRQ_n_sync ),
		.NMI_n   ( cpu_NMI_n_sync ),
		.BUSRQ_n ( 1'b1 ),
		.M1_n    ( cpu_m1_n ),
		.MREQ_n  ( cpu_mreq_n ),
		.IORQ_n  ( cpu_iorq_n ),
		.RD_n    ( cpu_rd_n ),
		.WR_n    ( cpu_wr_n ),
		.RFSH_n  (  ),
		.HALT_n  (  ),
		.BUSAK_n (  ),
		.A       ( cpu_addr ),
		.DI      ( cpu_din ),
		.DO      ( cpu_dout )
	);

	// Acorn Tube ULA
	tube inst_tube (
		.h_addr     ( h_addr ),
		.h_cs_b     ( h_cs_b ),
		.h_data_in  ( h_data_in ),
		.h_data_out ( h_data_out ),
		.h_phi2     ( h_clk ),
		.h_rdnw     ( h_rdnw ),
		.h_rst_b    ( h_rst_b ),
		.h_irq_b    ( h_irq_b ),
		.p_addr     ( cpu_addr[2:0] ),
		.p_cs_b     ( tube_cs_b ),               // ONE-cycle strobe per Z80 I/O access
		.p_data_in  ( cpu_dout ),
		.p_data_out ( p_data_out ),
		.p_rdnw     ( cpu_wr_n ),                // read = WR_n high, write = WR_n low
		.p_phi2     ( clk_cpu ),                 // 48 MHz, same as the host side
		.p_rst_b    ( RSTn ),
		.p_nmi_b    ( cpu_NMI_n ),
		.p_irq_b    ( cpu_IRQ_n )
	);

	// Tube parasite chip select: Z80 I/O space &00-&07 (IORQ active, MREQ idle).
	// This raw decode is asserted for the whole multi-T-state I/O cycle and
	// keeps feeding the CPU data-input mux below, so the T80 sees Tube data
	// during the entire access, exactly like the real ULA.
	assign p_cs_b = (cpu_mreq_n & cpu_m1_n & ~cpu_iorq_n & (cpu_addr[7:3] == 5'b00000)) ? 1'b0 : 1'b1;

	// One single-clk_cpu-cycle Tube select strobe per Z80 I/O access, fired on
	// the ASSERTION EDGE of the raw decode itself, plus read-data hold.
	//
	// Why the strobe: the gen_flag_v3 flag machines inside the Tube advance on
	// every p_phi2 (48 MHz) edge that samples select active. With the select
	// held for the whole access window (4-8 clk_cpu cycles), the FIFOs perform
	// repeated operations per instruction: the R3 two-byte flags alternate
	// (bytes vanish in pairs) and, below ~12 MHz, the req/ack round trip
	// (~100 ns) even fits inside the window. The one-shot presents the select
	// during exactly the FIRST clk_cpu cycle of the window - the same edge
	// where the previous ungated wiring performed its first (and correct)
	// operation - and suppresses the duplicates. Being anchored to the decode
	// edge, it does not depend on where CLKEN falls inside the window, i.e. it
	// makes no assumption about the T80 variant's bus-cycle tick alignment.
	//
	// Why the hold: the Tube's read-data output comes from hp_bytequad's mux,
	// which is qualified by the select (it outputs 8'bx when deselected), so
	// with a one-cycle select the read data is only valid during that cycle.
	// p_data_hold captures it at the strobe's closing edge (pre-pop value, mux
	// still selected) and holds it for the rest of the access, so the CPU
	// reads correct data wherever its DI sampling point falls - exactly the
	// real ULA behaviour of data remaining valid throughout the read strobe.
	assign tube_cs_b  = ~(p_cs_b_q & ~p_cs_b);
	assign p_data_cpu = p_hold_valid ? p_data_hold : p_data_out;

	always @(posedge clk_cpu) begin : tube_strobe
		p_cs_b_q <= p_cs_b;
		if (p_cs_b)
			p_hold_valid <= 1'b0;           // access finished: re-arm
		else if (~tube_cs_b) begin
			p_data_hold  <= p_data_out;     // pre-pop value at the strobe edge
			p_hold_valid <= 1'b1;
		end
	end

	// Boot ROM paged in for memory reads while bootmode is set
	assign rom_cs_b = (~cpu_mreq_n & ~cpu_rd_n & bootmode) ? 1'b0 : 1'b1;

	// RAM for all other memory accesses
	assign ram_cs_b = (~cpu_mreq_n & rom_cs_b) ? 1'b0 : 1'b1;

	// Z80 data input mux (CPU data in)
	assign cpu_din =
	  (~cpu_m1_n & ~cpu_iorq_n) ? 8'hfe :       // interrupt acknowledge -> RST 38
	  ~p_cs_b                   ? p_data_cpu :  // Tube data, held valid all access
	  ~rom_cs_b                 ? rom_data_out :
	  ~ram_cs_b                 ? ram_data_out :
	  8'hfe;

	// RAM bus
	assign ram_wr      = ~ram_cs_b & ~cpu_wr_n;
	assign ram_data_in = cpu_dout;
	assign ram_addr    = cpu_addr;

//------------------------------------------------------
// boot mode generator
//------------------------------------------------------
	always @(posedge clk_cpu) begin : boot_gen
		if (!RSTn_sync)
			bootmode <= 1'b1;
		else if (~cpu_mreq_n & ~cpu_m1_n) begin
			if (cpu_addr == 16'h0066)
				bootmode <= 1'b1;
			else if (cpu_addr[15])
				bootmode <= 1'b0;
		end
	end

//------------------------------------------------------
// power up reset
//------------------------------------------------------
	always @(posedge clk_cpu) begin : reset_gen
		if (!reset_counter[8])
			reset_counter <= reset_counter + 1'd1;
		RSTn_sync <= RSTn & reset_counter[8];
	end

//------------------------------------------------------
// interrupt synchronization
//------------------------------------------------------
	always @(posedge clk_cpu) begin : sync_gen
		if (!RSTn_sync) begin
			cpu_NMI_n_sync <= 1'b1;
			cpu_IRQ_n_sync <= 1'b1;
		end else if (cpu_clken) begin
			cpu_NMI_n_sync <= cpu_NMI_n;
			cpu_IRQ_n_sync <= cpu_IRQ_n;
		end
	end

	assign test = {RSTn, RSTn_sync, h_rst_b, cpu_NMI_n_sync, cpu_IRQ_n_sync, bootmode, 2'b00};

endmodule
