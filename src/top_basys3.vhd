library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;


-- Lab 4
entity top_basys3 is
    port(
        -- inputs
        clk     :   in std_logic; -- native 100MHz FPGA clock
        sw      :   in std_logic_vector(15 downto 0);
        btnU    :   in std_logic; -- master_reset
        btnL    :   in std_logic; -- clk_reset
        btnR    :   in std_logic; -- fsm_reset
        
        -- outputs
        led :   out std_logic_vector(15 downto 0);
        -- 7-segment display segments (active-low cathodes)
        seg :   out std_logic_vector(6 downto 0);
        -- 7-segment display active-low enables (anodes)
        an  :   out std_logic_vector(3 downto 0)
    );
end top_basys3;

architecture top_basys3_arch of top_basys3 is

    -- signal declarations
    signal s_reset_clk  : std_logic; -- resets clock divider(s)
    signal s_reset_fsm  : std_logic; -- synchronous reset for FSM

   -- divided clocks
    signal s_clk_elev   : std_logic; -- 2 Hz for elevator FSMs
    signal s_clk_disp   : std_logic; -- ≈1 kHz for seven-segment multiplex

   -- elevator floor outputs
    signal s_floorA     : std_logic_vector(3 downto 0);
    signal s_floorB     : std_logic_vector(3 downto 0);

   -- TD outputs
    signal s_hex_mux    : std_logic_vector(3 downto 0);
    signal s_an_mux     : std_logic_vector(3 downto 0);
    signal s_seg_n      : std_logic_vector(6 downto 0);
  
	-- component declarations
    component sevenseg_decoder is
        port (
            i_Hex : in STD_LOGIC_VECTOR (3 downto 0);
            o_seg_n : out STD_LOGIC_VECTOR (6 downto 0)
        );
    end component sevenseg_decoder;
    
    component elevator_controller_fsm is
		Port (
            i_clk        : in  STD_LOGIC;
            i_reset      : in  STD_LOGIC;
            is_stopped   : in  STD_LOGIC;
            go_up_down   : in  STD_LOGIC;
            o_floor : out STD_LOGIC_VECTOR (3 downto 0)		   
		 );
	end component elevator_controller_fsm;
	
	component TDM4 is
		generic ( constant k_WIDTH : natural  := 4); -- bits in input and output
        Port ( i_clk		: in  STD_LOGIC;
           i_reset		: in  STD_LOGIC; -- asynchronous
           i_D3 		: in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
		   i_D2 		: in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
		   i_D1 		: in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
		   i_D0 		: in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
		   o_data		: out STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
		   o_sel		: out STD_LOGIC_VECTOR (3 downto 0)	-- selected data line (one-cold)
	   );
    end component TDM4;
     
	component clock_divider is
        generic ( constant k_DIV : natural := 2	); -- How many clk cycles until slow clock toggles
                                                   -- Effectively, you divide the clk double this 
                                                   -- number (e.g., k_DIV := 2 --> clock divider of 4)
        port ( 	i_clk    : in std_logic;
                i_reset  : in std_logic;		   -- asynchronous
                o_clk    : out std_logic		   -- divided (slow) clock
        );
    end component clock_divider;
	
begin
	-- PORT MAPS ----------------------------------------
   s_reset_clk <= btnU or btnL;   -- master OR clock-only reset
   s_reset_fsm <= btnU or btnR;   -- master OR FSM-only   reset

   -- clock dividers
   clk_div_elev : clock_divider
      generic map (k_DIV => 25_000_000)          -- 100 MHz ÷ 50 000 000  = 1 Hz toggle → 0.5 s period
      port map    (i_clk   => clk,
                   i_reset => s_reset_clk,
                   o_clk   => s_clk_elev);

   clk_div_disp : clock_divider
      generic map (k_DIV => 100_000)             -- ≈500 µs toggle → ≈1 kHz refresh
      port map    (i_clk   => clk,
                   i_reset => s_reset_clk,
                   o_clk   => s_clk_disp);

   -- elevator FSM #1  (switches 0 = up/down, 1 = stop)
   elev_A : elevator_controller_fsm
      port map ( i_clk       => s_clk_elev,
                 i_reset     => s_reset_fsm,
                 is_stopped  => sw(0),
                 go_up_down  => sw(1),
                 o_floor     => s_floorA);

   ------------------------------------------------------------------
   -- elevator FSM #2  (switches 15 = up/down, 14 = stop)
   ------------------------------------------------------------------
   elev_B : elevator_controller_fsm
      port map ( i_clk       => s_clk_elev,
                 i_reset     => s_reset_fsm,
                 is_stopped  => sw(14),
                 go_up_down  => sw(15),
                 o_floor     => s_floorB);

   ------------------------------------------------------------------
   -- 4-way time-division multiplexer for the seven-segment display
   --   D3  = constant F  (unused display furthest left)
   --   D2  = elevator-B floor  (second from left)
   --   D1  = constant F  (unused display second from right)
   --   D0  = elevator-A floor  (right-most display)
   ------------------------------------------------------------------
   disp_mux : TDM4
      port map ( i_clk  => s_clk_disp,
                 i_reset=> s_reset_clk,
                 i_D3   => x"F",
                 i_D2   => s_floorB,
                 i_D1   => x"F",
                 i_D0   => s_floorA,
                 o_data => s_hex_mux,
                 o_sel  => s_an_mux);

   ------------------------------------------------------------------
   -- hexadecimal-to-7-segment decoder
   ------------------------------------------------------------------
   seg_dec : sevenseg_decoder
      port map ( i_Hex   => s_hex_mux,
                 o_seg_n => s_seg_n);

   -- drive Basys 3 outputs
   seg <= s_seg_n;          -- active-low segments
   an  <= s_an_mux;         -- active-low anodes

   ------------------------------------------------------------------
   -- LEDs
   ------------------------------------------------------------------
   led(15)           <= s_clk_elev;           -- requirement: LED 15 shows FSM clock
   led(14 downto 0)  <= (others => '0');      -- ground remaining LEDs
	
	-- CONCURRENT STATEMENTS ----------------------------
	
	-- LED 15 gets the FSM slow clock signal. The rest are grounded.
	
	-- leave unused switches UNCONNECTED. Ignore any warnings this causes.
	
	-- reset signals
	
end top_basys3_arch;
