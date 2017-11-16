library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity if_id is
  port(CLK            : in  std_logic;
  		id_flush, id_stall, ifid_reset : in std_logic;
       	if_instruction  : in std_logic_vector(31 downto 0);
       	id_instruction  : out std_logic_vector(31 downto 0);
       	if_pc_plus_4 : in std_logic_vector(31 downto 0);
       	id_pc_plus_4 : out std_logic_vector(31 downto 0));
 end if_id;

architecture mixed of if_id is 

begin


write_process : process(CLK)
begin

	if (rising_edge(CLK)) then
		if(id_flush = '1' or ifid_reset = '1') then
			id_instruction <= (others => '0');
			id_pc_plus_4 <= (others => '0');

		elsif(id_stall = '0') then
			id_instruction <= if_instruction;
			id_pc_plus_4 <= if_pc_plus_4;
		end if;
	end if;

end process write_process;

end mixed;