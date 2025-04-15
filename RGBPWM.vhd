library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;  -- This is the preferred library for arithmetic types

entity RGBPWM is
  port (
    clk : in std_logic;
    rst : in std_logic;
    --dynamic_bits : in integer := 8;  -- Input for dynamic bit width
    ColorData : in std_logic_vector(3 downto 0);
    PWMout : out std_logic
  );
end RGBPWM;

architecture Behavioral of RGBPWM is
    signal counter : integer := 0;  -- Counter with max possible size
    signal count_max : integer := 15;  -- Maximum value for the counter based on ColorData width
begin


    -- Main clocked process to handle PWM logic
    process(clk, rst)
    begin
        if rst = '1' then
            counter <= 0;
            PWMout <= '0';  -- Initialize PWMout during reset
        elsif rising_edge(clk) then
            if counter >= count_max then
                counter <= 0;  -- Reset counter when it reaches count_max
            else
                counter <= counter + 1;
            end if;

            -- Compare counter with truncated SRAM data to generate PWM signal
            if (counter < to_integer(unsigned(ColorData))) then
                PWMout <= '1';
            else
                PWMout <= '0';
            end if;
        end if;
    end process;

end Behavioral;
