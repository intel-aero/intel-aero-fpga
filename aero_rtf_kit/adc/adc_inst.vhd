	component adc is
		port (
			adc_adc_pll_clock_clk       : in  std_logic                     := 'X';             -- clk
			adc_adc_pll_locked_export   : in  std_logic                     := 'X';             -- export
			adc_response_valid          : out std_logic;                                        -- valid
			adc_response_channel        : out std_logic_vector(4 downto 0);                     -- channel
			adc_response_data           : out std_logic_vector(11 downto 0);                    -- data
			adc_response_startofpacket  : out std_logic;                                        -- startofpacket
			adc_response_endofpacket    : out std_logic;                                        -- endofpacket
			adc_sequencer_csr_address   : in  std_logic                     := 'X';             -- address
			adc_sequencer_csr_read      : in  std_logic                     := 'X';             -- read
			adc_sequencer_csr_write     : in  std_logic                     := 'X';             -- write
			adc_sequencer_csr_writedata : in  std_logic_vector(31 downto 0) := (others => 'X'); -- writedata
			adc_sequencer_csr_readdata  : out std_logic_vector(31 downto 0);                    -- readdata
			clk_clk                     : in  std_logic                     := 'X';             -- clk
			reset_reset_n               : in  std_logic                     := 'X'              -- reset_n
		);
	end component adc;

	u0 : component adc
		port map (
			adc_adc_pll_clock_clk       => CONNECTED_TO_adc_adc_pll_clock_clk,       --  adc_adc_pll_clock.clk
			adc_adc_pll_locked_export   => CONNECTED_TO_adc_adc_pll_locked_export,   -- adc_adc_pll_locked.export
			adc_response_valid          => CONNECTED_TO_adc_response_valid,          --       adc_response.valid
			adc_response_channel        => CONNECTED_TO_adc_response_channel,        --                   .channel
			adc_response_data           => CONNECTED_TO_adc_response_data,           --                   .data
			adc_response_startofpacket  => CONNECTED_TO_adc_response_startofpacket,  --                   .startofpacket
			adc_response_endofpacket    => CONNECTED_TO_adc_response_endofpacket,    --                   .endofpacket
			adc_sequencer_csr_address   => CONNECTED_TO_adc_sequencer_csr_address,   --  adc_sequencer_csr.address
			adc_sequencer_csr_read      => CONNECTED_TO_adc_sequencer_csr_read,      --                   .read
			adc_sequencer_csr_write     => CONNECTED_TO_adc_sequencer_csr_write,     --                   .write
			adc_sequencer_csr_writedata => CONNECTED_TO_adc_sequencer_csr_writedata, --                   .writedata
			adc_sequencer_csr_readdata  => CONNECTED_TO_adc_sequencer_csr_readdata,  --                   .readdata
			clk_clk                     => CONNECTED_TO_clk_clk,                     --                clk.clk
			reset_reset_n               => CONNECTED_TO_reset_reset_n                --              reset.reset_n
		);

