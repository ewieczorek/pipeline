library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity pipeline_register_template is
  port(CLK           : in  std_logic;
  		  in_flush, in_stall, in_reset : in std_logic;
  		  in_instruction  : in std_logic_vector(31 downto 0); -- pass instruction along (useful for debugging)
        out_instruction  : out std_logic_vector(31 downto 0);
        in_pc_plus_4: in std_logic_vector(31 downto 0);
        out_pc_plus_4  : out std_logic_vector(31 downto 0);

  	-- CONTROL signals
        in_reg_dest   : in std_logic;
  	    in_branch 	 : in std_logic;
  	    in_mem_to_reg : in std_logic;
  	    in_ALU_op 	 : in std_logic_vector(3 downto 0);
  	    in_mem_write  : in std_logic;
  	    in_ALU_src 	 : in std_logic;
  	    in_reg_write  : in std_logic;
  	    out_reg_dest   : out std_logic;
  	    out_branch 	 : out std_logic;
  	    out_mem_to_reg : out std_logic;
  	    out_ALU_op 	 : out std_logic_vector(3 downto 0);
  	    out_mem_write  : out std_logic;
  	    out_ALU_src 	 : out std_logic;
  	    out_reg_write  : out std_logic;
  	-- END CONTROL signals

  	-- Register signals
  		in_rs_data : in std_logic_vector(31 downto 0);
  		in_rt_data : in std_logic_vector(31 downto 0);
  		out_rs_data : out std_logic_vector(31 downto 0);
  		out_rt_data : out std_logic_vector(31 downto 0);
  		in_rs_sel : in std_logic_vector(4 downto 0);
  		in_rt_sel : in std_logic_vector(4 downto 0);
      in_rd_sel : in std_logic_vector(4 downto 0);
  		out_rs_sel : out std_logic_vector(4 downto 0);
  		out_rt_sel : out std_logic_vector(4 downto 0);
      out_rd_sel : out std_logic_vector(4 downto 0)
  	-- END Register signals

  	-- other signals go here
  	    );
end pipeline_register_template;

architecture mixed of pipeline_register_template is 

begin

write_process : process(CLK)
begin

	if (rising_edge(CLK)) then
		if(in_flush = '1' or in_reset = '1') then
			-- fill in reset/flush operation here (i.e. set outputs to 0)
		elsif(in_stall = '0') then
			-- fill in normal operation here
		end if;
	end if;

end process write_process;

end mixed;