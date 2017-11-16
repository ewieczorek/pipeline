library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity id_ex is
  port(CLK           : in  std_logic;
  		ex_flush, ex_stall, idex_reset : in std_logic;
  		id_instruction  : in std_logic_vector(31 downto 0); -- pass instruction along (useful for debugging)
        ex_instruction  : out std_logic_vector(31 downto 0);
        id_pc_plus_4 : in std_logic_vector(31 downto 0);
       	ex_pc_plus_4 : out std_logic_vector(31 downto 0);

  	-- CONTROL signals
        id_reg_dest   : in std_logic;
  	    id_branch 	 : in std_logic;
  	    id_mem_to_reg : in std_logic;
  	    id_ALU_op 	 : in std_logic_vector(3 downto 0);
  	    id_mem_write  : in std_logic;
  	    id_ALU_src 	 : in std_logic;
  	    id_reg_write  : in std_logic;
  	    ex_reg_dest   : out std_logic;
  	    ex_branch 	 : out std_logic;
  	    ex_mem_to_reg : out std_logic;
  	    ex_ALU_op 	 : out std_logic_vector(3 downto 0);
  	    ex_mem_write  : out std_logic;
  	    ex_ALU_src 	 : out std_logic;
  	    ex_reg_write  : out std_logic;
  	-- END CONTROL signals

  	-- Register signals
  		id_rs_data : in std_logic_vector(31 downto 0);
  		id_rt_data : in std_logic_vector(31 downto 0);
  		ex_rs_data : out std_logic_vector(31 downto 0);
  		ex_rt_data : out std_logic_vector(31 downto 0);
  		id_rs_sel : in std_logic_vector(4 downto 0);
  		id_rt_sel : in std_logic_vector(4 downto 0);
  		id_rd_sel : in std_logic_vector(4 downto 0);
  		ex_rs_sel : out std_logic_vector(4 downto 0);
  		ex_rt_sel : out std_logic_vector(4 downto 0);
  		ex_rd_sel : out std_logic_vector(4 downto 0);
  	-- END Register signals

  		id_extended_immediate : in std_logic_vector(31 downto 0);
  		ex_extended_immediate : out std_logic_vector(31 downto 0)
  	    );
end id_ex;

architecture mixed of id_ex is 

begin

write_process : process(CLK)
begin

	if (rising_edge(CLK)) then
		if(ex_flush = '1' or idex_reset = '1') then
			ex_instruction <= (others => '0');
			ex_pc_plus_4 <= (others => '0');

			ex_reg_dest <= '0';
			ex_branch <= '0';
			ex_mem_to_reg <= '0';
			ex_ALU_op <= (others => '0');
			ex_mem_write <= '0';
			ex_ALU_src <= '0';
			ex_reg_write <= '0';

			ex_rs_data <= (others => '0');
	  		ex_rt_data <= (others => '0');
	  		ex_rs_sel <= (others => '0');
	  		ex_rt_sel <= (others => '0');
	  		ex_rd_sel <= (others => '0');

	  		ex_extended_immediate <= (others => '0');

		elsif(ex_stall = '0') then
			ex_instruction <= id_instruction;
			ex_pc_plus_4 <= id_pc_plus_4;

			ex_reg_dest <= id_reg_dest;
			ex_branch <= id_branch;
			ex_mem_to_reg <= id_mem_to_reg;
			ex_ALU_op <= id_ALU_op;
			ex_mem_write <= id_mem_write;
			ex_ALU_src <= id_ALU_src;
			ex_reg_write <= id_reg_write;

			ex_rs_data <= id_rs_data;
	  		ex_rt_data <= id_rt_data;
	  		ex_rs_sel <= id_rs_sel;
	  		ex_rt_sel <= id_rt_sel;
	  		ex_rd_sel <= id_rd_sel;

	  		ex_extended_immediate <= id_extended_immediate;

		end if;

	end if;

end process write_process;

end mixed;