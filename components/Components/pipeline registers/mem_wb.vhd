library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity mem_wb is
  port(CLK           : in  std_logic;
		wb_flush, wb_stall, memwb_reset : in std_logic;
		mem_instruction  : in std_logic_vector(31 downto 0); -- pass instruction along (useful for debugging)
        wb_instruction  : out std_logic_vector(31 downto 0);
        mem_pc_plus_4 : in std_logic_vector(31 downto 0);
       	wb_pc_plus_4 : out std_logic_vector(31 downto 0);

  	-- CONTROL signals
        mem_reg_dest   : in std_logic;
  	    mem_mem_to_reg : in std_logic;
  	    mem_reg_write  : in std_logic;
  	    wb_reg_dest   : out std_logic;
  	    wb_mem_to_reg : out std_logic;
  	    wb_reg_write  : out std_logic;
  	-- END CONTROL signals

  	-- ALU signals
		mem_ALU_out : in std_logic_vector(31 downto 0);
		wb_ALU_out : out std_logic_vector(31 downto 0);
  	-- END ALU signals

  	-- Memory signals
		mem_dmem_out : in std_logic_vector(31 downto 0);
		wb_dmem_out : out std_logic_vector(31 downto 0);
  	-- END Memory signals

	-- Register signals
  		mem_write_reg_sel : in std_logic_vector(4 downto 0);
  		wb_write_reg_sel : out std_logic_vector(4 downto 0)
  	-- END Register signals
  	    );
end mem_wb;

architecture mixed of mem_wb is 

begin

write_process : process(CLK)
begin

	if (rising_edge(CLK)) then
		if(wb_flush = '1' or memwb_reset = '1') then
			wb_instruction <= (others => '0');
			wb_pc_plus_4 <= (others => '0');

			wb_reg_dest <= '0';
			wb_mem_to_reg <= '0';
			wb_reg_write <= '0';

			wb_ALU_out <= (others => '0');	

			wb_dmem_out <= (others => '0');	

		  	wb_write_reg_sel <= (others => '0');

		elsif(wb_stall = '0') then
			wb_instruction <= mem_instruction;
			wb_pc_plus_4 <= mem_pc_plus_4;

			wb_reg_dest <= mem_reg_dest;
			wb_mem_to_reg <= mem_mem_to_reg;
			wb_reg_write <= mem_reg_write;

			wb_ALU_out <= mem_ALU_out;

			wb_dmem_out <= mem_dmem_out;

		  	wb_write_reg_sel <= mem_write_reg_sel;

		end if;

	end if;

end process write_process;

end mixed;