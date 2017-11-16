library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity ex_mem is
  port(CLK           : in  std_logic;
		mem_flush, mem_stall, exmem_reset : in std_logic;
		ex_instruction  : in std_logic_vector(31 downto 0); -- pass instruction along (useful for debugging)
        mem_instruction  : out std_logic_vector(31 downto 0);
        ex_pc_plus_4 : in std_logic_vector(31 downto 0);
       	mem_pc_plus_4 : out std_logic_vector(31 downto 0);

  	-- CONTROL signals
        ex_reg_dest   : in std_logic;
  	    ex_mem_to_reg : in std_logic;
  	    ex_mem_write  : in std_logic;
  	    ex_reg_write  : in std_logic;
  	    mem_reg_dest   : out std_logic;
  	    mem_mem_to_reg : out std_logic;
  	    mem_mem_write  : out std_logic;
  	    mem_reg_write  : out std_logic;
  	-- END CONTROL signals

  	-- ALU signals
		ex_ALU_out : in std_logic_vector(31 downto 0);
		mem_ALU_out : out std_logic_vector(31 downto 0);
  	-- END ALU signals

	-- Register signals
		ex_rt_data : in std_logic_vector(31 downto 0);
		mem_rt_data : out std_logic_vector(31 downto 0);
  		ex_write_reg_sel : in std_logic_vector(4 downto 0); -- see the Reg. Dest. mux in the pipeline archteicture diagram
  		mem_write_reg_sel : out std_logic_vector(4 downto 0)
  	-- END Register signals
  	    );
end ex_mem;

architecture mixed of ex_mem is 

begin

write_process : process(CLK)
begin

	if (rising_edge(CLK)) then
		if(mem_flush = '1' or exmem_reset = '1') then
			mem_instruction <= (others => '0');
			mem_pc_plus_4 <= (others => '0');

			mem_reg_dest <= '0';
			mem_mem_to_reg <= '0';
			mem_mem_write <= '0';
			mem_reg_write <= '0';

			mem_ALU_out <= (others => '0');			

		  	mem_rt_data <= (others => '0');
		  	mem_write_reg_sel <= (others => '0');

		elsif(mem_stall = '0') then
			mem_instruction <= ex_instruction;
			mem_pc_plus_4 <= ex_pc_plus_4;

			mem_reg_dest <= ex_reg_dest;
			mem_mem_to_reg <= ex_mem_to_reg;
			mem_mem_write <= ex_mem_write;
			mem_reg_write <= ex_reg_write;

			mem_ALU_out <= ex_ALU_out;

		  	mem_rt_data <= ex_rt_data;
		  	mem_write_reg_sel <= ex_write_reg_sel;

		end if;

	end if;

end process write_process;

end mixed;