library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity PC_reg is
  port(	CLK : in std_logic;
  		reset : in std_logic;
  		stall : in std_logic;
  		i_next_PC : in std_logic_vector(31 downto 0);
  	   	o_PC : out std_logic_vector(31 downto 0));
 end PC_reg;

architecture mixed of PC_reg is 

begin

process(CLK, reset)
begin

	if(reset = '1') then
		o_PC <= (others => '0');
	elsif (rising_edge(CLK)) then
		if(stall = '0') then
			o_PC <= i_next_PC;
		end if;
	end if;

end process;

end mixed;