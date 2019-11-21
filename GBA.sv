//============================================================================
//  GBA
//  Copyright (C) 2019 Robert Peip
//
//  Port to MiSTer
//  Copyright (C) 2019 Sorgelig
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================ 

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

assign ADC_BUS  = 'Z;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign USER_OUT = '1;

assign AUDIO_S   = 1;
assign AUDIO_MIX = 0;

assign LED_USER  = cart_download & bk_pending;
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

assign VIDEO_ARX = status[1] ? 8'd16 : 8'd3;
assign VIDEO_ARY = status[1] ? 8'd9  : 8'd2;

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

///////////////////////  CLOCK/RESET  ///////////////////////////////////

wire pll_locked;
wire clk_sys;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(SDRAM_CLK),
	.locked(pll_locked)
);

wire reset = RESET | buttons[1] | status[0] | cart_download | bk_loading;

////////////////////////////  HPS I/O  //////////////////////////////////

`include "build_id.v"
parameter CONF_STR = {
    "GBA;;",
    "FS,GBA;",
    "-;",
    //"C,Cheats;",
    //"H1OO,Cheats Enabled,Yes,No;",
    //"-;",
    //"D0RC,Load Backup RAM;",
    //"D0RD,Save Backup RAM;",
    //"D0ON,Autosave,Off,On;",
    //"D0-;",
    "O1,Aspect Ratio,3:2,16:9;",
    //"O9B,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
    //"OJK,Stereo Mix,None,25%,50%,100%;", 
    "-;",
    "R0,Reset;",
    "J1,A,B,L,R,Select,Start,Turbo;",
	 "jn,A,B,L,R,Select,Start,X;",
    "V,v",`BUILD_DATE
};

wire  [1:0] buttons;
wire [31:0] status;
wire [15:0] status_menumask = {~gg_available, ~bk_ena};
wire        forced_scandoubler;
reg  [31:0] sd_lba;
reg         sd_rd = 0;
reg         sd_wr = 0;
wire        sd_ack;
wire  [7:0] sd_buff_addr;
wire [15:0] sd_buff_dout;
wire [15:0] sd_buff_din;
wire        sd_buff_wr;
wire        img_mounted;
wire        img_readonly;
wire [63:0] img_size;
wire        ioctl_download;
wire [24:0] ioctl_addr;
wire [15:0] ioctl_dout;
wire        ioctl_wr;
wire  [7:0] ioctl_index;

wire [11:0] joy;
wire [21:0] gamma_bus;

hps_io #(.STRLEN($size(CONF_STR)>>3), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.conf_str(CONF_STR),

	.buttons(buttons),
	.forced_scandoubler(forced_scandoubler),

	.joystick_0(joy),

	.status(status),
	.status_menumask(status_menumask),

	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wr(ioctl_wr),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	.img_mounted(img_mounted),
	.img_readonly(img_readonly),
	.img_size(img_size),
	
	.gamma_bus(gamma_bus)
);

//////////////////////////  ROM DETECT  /////////////////////////////////
/*
reg        PAL;
reg  [7:0] rom_type;
reg [23:0] rom_mask, ram_mask;
always @(posedge clk_sys) begin
	reg [3:0] rom_size;
	reg [3:0] ram_size;
	reg       rom_region = 0;

	if (cart_download) begin
		if(ioctl_wr) begin
			if (ioctl_addr == 0) begin
				rom_size <= 4'hC;
				ram_size <= 4'h0;
				if(!LHRom_type && ioctl_dout[7:0]) {ram_size,rom_size} <= ioctl_dout[7:0];

				case(LHRom_type)
					1: rom_type <= 0;
					2: rom_type <= 0;
					3: rom_type <= 1;
					4: rom_type <= 2;
					default: rom_type <= ioctl_dout[15:8];
				endcase
			end

			if (ioctl_addr == 2) begin
				rom_region <= ioctl_dout[8];
			end

			if(LHRom_type == 2) begin
				if(ioctl_addr == ('h7FD6+'h200)) rom_size <= ioctl_dout[11:8];
				if(ioctl_addr == ('h7FD8+'h200)) ram_size <= ioctl_dout[3:0];
			end
			else if(LHRom_type == 3) begin
				if(ioctl_addr == ('hFFD6+'h200)) rom_size <= ioctl_dout[11:8];
				if(ioctl_addr == ('hFFD8+'h200)) ram_size <= ioctl_dout[3:0];
			end
			else if(LHRom_type == 4) begin
				if(ioctl_addr == ('h40FFD6+'h200)) rom_size <= ioctl_dout[11:8];
				if(ioctl_addr == ('h40FFD8+'h200)) ram_size <= ioctl_dout[3:0];
			end

			rom_mask <= (24'd1024 << rom_size) - 1'd1;
			ram_mask <= ram_size ? (24'd1024 << ram_size) - 1'd1 : 24'd0;
		end
	end
	else begin
		PAL <= (!status[15:14]) ? rom_region : status[15];
	end
end
*/

wire code_index = &ioctl_index;
wire code_download = ioctl_download & code_index;
wire cart_download = ioctl_download & ~code_index; 

reg [24:0] last_addr;
always @(posedge clk_sys) begin
	reg old_download;
	
	old_download <= cart_download;
	if (old_download & ~cart_download) last_addr <= ioctl_addr;
end

////////////////////////////  SYSTEM  ///////////////////////////////////

gba_top
#(
	.Softmap_GBA_Gamerom_ADDR(0),
	.Softmap_GBA_WRam_ADDR  (8388608),             //  65536 (32bit) -- 256 Kbyte Data for GBA WRam Large
	.Softmap_GBA_FLASH_ADDR (8388608+65536),       // 131072 (8bit)  -- 128 Kbyte Data for GBA Flash
	.Softmap_GBA_EEPROM_ADDR(8388608+65536+131072) //   8192 (8bit)  --   8 Kbyte Data for GBA EEProm
)
gba
(
	.clk100(clk_sys),
	.GBA_on(~reset),                  // switching from off to on = reset
	.GBA_lockspeed(~joy[10]),         // 1 = 100% speed, 0 = max speed
	.GBA_flash_1m(0),                 // 1 when string "FLASH1M_V" is anywhere in gamepak
	.CyclePrecalc(100),               // 100 seems to be ok to keep fullspeed for all games
	.MaxPakAddr(last_addr[24:2]),     // max byte address that will contain data, required for buggy games that read behind their own memory, e.g. zelda minish cap
	.CyclesMissing(),                 // debug only for speed measurement, keep open

	.sdram_read_ena(sdram_req),       // triggered once for read request 
	.sdram_read_done(sdram_ack),      // must be triggered once when sdram_read_data is valid after last read
	.sdram_read_addr(sdram_addr),     // all addresses are DWORD addresses!
	.sdram_read_data(sdram_dout1),    // data from last request, valid when done = 1
	.sdram_second_dword(sdram_dout2), // second dword to be read for buffering/prefetch. Must be valid 1 cycle after done = 1

	.bus_out_Din(bus_din),            // data read from WRam Large, SRAM/Flash/EEPROM
	.bus_out_Dout(bus_dout),          // data written to WRam Large, SRAM/Flash/EEPROM
	.bus_out_Adr(bus_addr),           // all addresses are DWORD addresses!
	.bus_out_rnw(bus_rd),             // read = 1, write = 0
	.bus_out_ena(bus_req),            // one cycle high for each action
	.bus_out_done(bus_ack),           // should be one cycle high when write is done or read value is valid

	.KeyA(joy[4]),
	.KeyB(joy[5]),
	.KeySelect(joy[8]),
	.KeyStart(joy[9]),
	.KeyRight(joy[0]),
	.KeyLeft(joy[1]),
	.KeyUp(joy[3]),
	.KeyDown(joy[2]),
	.KeyR(joy[7]),
	.KeyL(joy[6]),
	
	.pixel_out_addr(pixel_addr),      // integer range 0 to 38399;       -- address for framebuffer 
	.pixel_out_data(pixel_data),      // RGB data for framebuffer 
	.pixel_out_we(pixel_we),          // new pixel for framebuffer 

	.sound_out(AUDIO_L)
);

assign AUDIO_R = AUDIO_L;

////////////////////////////  CODES  ///////////////////////////////////

reg [128:0] gg_code;
wire gg_available;

// Code layout:
// {clock bit, code flags,     32'b address, 32'b compare, 32'b replace}
//  128        127:96          95:64         63:32         31:0
// Integer values are in BIG endian byte order, so it up to the loader
// or generator of the code to re-arrange them correctly.

always_ff @(posedge clk_sys) begin
	gg_code[128] <= 0;

	if (code_download & ioctl_wr) begin
		case (ioctl_addr[3:0])
			0:  gg_code[111:96]  <= ioctl_dout; // Flags Bottom Word
			2:  gg_code[127:112] <= ioctl_dout; // Flags Top Word
			4:  gg_code[79:64]   <= ioctl_dout; // Address Bottom Word
			6:  gg_code[95:80]   <= ioctl_dout; // Address Top Word
			8:  gg_code[47:32]   <= ioctl_dout; // Compare Bottom Word
			10: gg_code[63:48]   <= ioctl_dout; // Compare top Word
			12: gg_code[15:0]    <= ioctl_dout; // Replace Bottom Word
			14: begin
				gg_code[31:16]    <= ioctl_dout; // Replace Top Word
				gg_code[128]      <= 1;          // Clock it in
			end
		endcase
	end
end

////////////////////////////  MEMORY  ///////////////////////////////////

wire [25:2] sdram_addr;
wire [31:0] sdram_dout1, sdram_dout2;
wire        sdram_req, sdram_ack;

wire [25:2] bus_addr;
wire [31:0] bus_dout, bus_din;
wire        bus_rd, bus_req, bus_ack;

sdram sdram
(
	.*,
	.init(~pll_locked),
	.clk(clk_sys),

	.ch1_addr(cart_download ? ioctl_addr[24:1] : {sdram_addr, 1'b0}),
	.ch1_din(ioctl_dout),
	.ch1_dout({sdram_dout2, sdram_dout1}),
	.ch1_req(cart_download ? ioctl_wr : sdram_req),
	.ch1_rnw(cart_download ? 1'b0     : 1'b1     ),
	.ch1_ready(sdram_ack),

	.ch2_addr({bus_addr, 1'b0}),
	.ch2_din(bus_din),
	.ch2_dout(bus_dout),
	.ch2_req(~cart_download & bus_req),
	.ch2_rnw(bus_rd),
	.ch2_ready(bus_ack)
);

wire [15:0] pixel_addr, px_addr;
wire [14:0] pixel_data, rgb;
wire        pixel_we;

dpram_n #(16,15,38400) vram
(
	.clock(clk_sys),

	.address_a(pixel_addr),
	.data_a(pixel_data),
	.wren_a(pixel_we),

	.address_b(px_addr),
	.q_b(rgb)
);

reg hs, vs, hbl, vbl, ce_pix;
reg [4:0] r,g,b;
always @(posedge clk_sys) begin
	reg [7:0] x,y;
	reg [1:0] div;
	reg sync;

	if(pixel_we && pixel_addr == 38399) sync <= 1;

	div <= div + 1'd1;

	ce_pix <= 0;
	if(!div) begin
		ce_pix <= 1;

		{r,g,b} <= rgb;

		hbl <= &x[7:4];
		if(x == 244) begin
			hs <= 1;

			if(y == 163) vs <= 1;
			if(y == 166) vs <= 0;
		end
		if(x == 252) hs  <= 0;

		if(y == 160) vbl <= 1;
		if(y == 000) vbl <= 0;
	end

	if(ce_pix) begin
		if(!hbl) px_addr <= px_addr + 1'd1;
		if(~&y) begin
			x <= x + 1'd1;
			if(&x) y <= y + 1'd1;
		end
		else if(sync) begin
			sync <= 0;
			px_addr <= 0;
			x <= 0;
			y <= 0;
		end
	end
end

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL = ce_pix;

assign VGA_R  = {r,r[4:2]};
assign VGA_G  = {g,g[4:2]};
assign VGA_B  = {b,b[4:2]};
assign VGA_DE = ~(hbl | vbl);
assign VGA_HS = hs;
assign VGA_VS = vs;
assign VGA_F1 = 0;
assign VGA_SL = 0;


////////////////////////////  VIDEO  ////////////////////////////////////
/*
wire [7:0] R,G,B;
wire FIELD,INTERLACE;
wire HSync, HSYNC;
wire VSync, VSYNC;
wire HBlank_n;
wire VBlank_n;
wire HIGH_RES;
wire DOTCLK;

reg interlace;
reg ce_pix;
always @(posedge CLK_VIDEO) begin
	reg [2:0] pcnt;
	reg old_vsync;
	reg tmp_hres, frame_hres;
	reg old_dotclk;
	
	tmp_hres <= tmp_hres | HIGH_RES;

	old_vsync <= VSync;
	if(~old_vsync & VSync) begin
		frame_hres <= tmp_hres | ~scandoubler;
		tmp_hres <= HIGH_RES;
		interlace <= INTERLACE;
	end

	pcnt <= pcnt + 1'd1;
	old_dotclk <= DOTCLK;
	if(~old_dotclk & DOTCLK & HBlank_n & VBlank_n) pcnt <= 1;

	ce_pix <= !pcnt[1:0] & (frame_hres | ~pcnt[2]);
	
	if(pcnt==3) {HSync, VSync} <= {HSYNC, VSYNC};
end

assign VGA_F1 = 0;
assign VGA_SL = sl[1:0];

wire [2:0] scale = status[11:9];
wire [2:0] sl = scale ? scale - 1'd1 : 3'd0;
wire       scandoubler = ~interlace && (scale || forced_scandoubler);

video_mixer #(.LINE_LENGTH(520), .GAMMA(1)) video_mixer
(
	.*,

	.clk_vid(CLK_VIDEO),
	.ce_pix_out(CE_PIXEL),

	.scanlines(0),
	.hq2x(scale==1),
	.mono(0),

	.HBlank(~HBlank_n),
	.VBlank(~VBlank_n),
	.R(R),
	.G(G),
	.B(B)
);
*/

/////////////////////////  STATE SAVE/LOAD  /////////////////////////////
reg bk_ena = 0;
reg bk_pending = 0;
reg bk_loading = 0;

/*
wire bk_save_write = ~BSRAM_CE_N & ~BSRAM_WE_N;

always @(posedge clk_sys) begin
	if (bk_ena && ~OSD_STATUS && bk_save_write)
		bk_pending <= 1'b1;
	else if (bk_state)
		bk_pending <= 1'b0;
end

reg old_downloading = 0;
always @(posedge clk_sys) begin
	old_downloading <= cart_download;
	if(~old_downloading & cart_download) bk_ena <= 0;
	
	//Save file always mounted in the end of downloading state.
	if(cart_download && img_mounted && !img_readonly) bk_ena <= |ram_mask;
end

wire bk_load    = status[12];
wire bk_save    = status[13] | (bk_pending & OSD_STATUS && status[23]);
reg  bk_state   = 0;

always @(posedge clk_sys) begin
	reg old_load = 0, old_save = 0, old_ack;

	old_load <= bk_load & bk_ena;
	old_save <= bk_save & bk_ena;
	old_ack  <= sd_ack;

	if(~old_ack & sd_ack) {sd_rd, sd_wr} <= 0;
	
	if(!bk_state) begin
		if((~old_load & bk_load) | (~old_save & bk_save)) begin
			bk_state <= 1;
			bk_loading <= bk_load;
			sd_lba <= 0;
			sd_rd <=  bk_load;
			sd_wr <= ~bk_load;
		end
		if(old_downloading & ~cart_download & |img_size & bk_ena) begin
			bk_state <= 1;
			bk_loading <= 1;
			sd_lba <= 0;
			sd_rd <= 1;
			sd_wr <= 0;
		end
	end else begin
		if(old_ack & ~sd_ack) begin
			if(sd_lba >= ram_mask[23:9]) begin
				bk_loading <= 0;
				bk_state <= 0;
			end else begin
				sd_lba <= sd_lba + 1'd1;
				sd_rd  <=  bk_loading;
				sd_wr  <= ~bk_loading;
			end
		end
	end
end
*/
 
endmodule
