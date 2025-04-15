library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

entity LCDDataSelect is
  port (
    clk      : in std_logic;
    reset    : in std_logic;
    busy     : in std_logic                       := '0';
    data_in  : in std_logic_vector(255 downto 0);
    data_out : out std_logic_vector(7 downto 0) := (others => '0')
  );
end LCDDataSelect;

architecture Behavioral of LCDDataSelect is

  signal LCD_EN : std_logic := '0';
  signal LCD_RS : std_logic := '0';
  signal RS     : std_logic := '0';
  signal LCD_RW : std_logic := '0';
  signal LCD_BL : std_logic := '1';
  signal nibble : std_logic := '0';
  signal data   : std_logic_vector(7 downto 0);
  signal counter : integer := 0;
  signal LCD_DATA : std_logic_vector(3 downto 0);
  signal byteSel : integer range 1 to 43 := 1;
  signal oldBusy : std_logic;
  signal dataCount : integer range 1 to 6 := 1;
  signal prevDataCount : integer range 1 to 6 := 1;

begin
  
  LCD_RW <= '0';
  LCD_BL <= '1';
  data_out <= LCD_DATA & LCD_BL & LCD_EN & LCD_RW & LCD_RS;

  process (clk, reset)
  begin
    if (reset = '1') then
      byteSel <= 1;
      dataCount <= 1;
      prevDataCount <= 1;
      oldBusy <= '0';
      LCD_RS <= '0';
    elsif rising_edge(clk) then
      oldBusy <= busy;
      case prevDataCount is
          when 1 =>
            LCD_EN <= '0';
            LCD_DATA <= data(7 downto 4);
            LCD_RS <= RS;
          when 2 =>
            LCD_EN <= '1';
            LCD_DATA <= data(7 downto 4);
            LCD_RS <= RS;
          when 3 =>
            LCD_EN <= '0';
            LCD_DATA <= data(7 downto 4);
            LCD_RS <= RS;
          when 4 =>
            LCD_EN <= '0';
            LCD_DATA <= data(3 downto 0);
            LCD_RS <= RS;
          when 5 =>
            LCD_EN <= '1';
            LCD_DATA <= data(3 downto 0);
            LCD_RS <= RS;
          when 6 =>
            LCD_EN <= '0';
            LCD_DATA <= data(3 downto 0);
            LCD_RS <= RS;
          when others => 
            LCD_EN <= '0';
            LCD_DATA <= data(7 downto 4);
            LCD_RS <= RS;
        end case;

      if oldBusy = '1' and busy = '0' then
        if dataCount < 6 then
            dataCount <= dataCount + 1;
        else
            dataCount <= 1;
        end if;
        prevDataCount <= dataCount;
        if dataCount = 1 and prevDataCount = 6 then
          if byteSel < 42 then
            byteSel <= byteSel + 1;
            --LCD_RS <= RS; bad, updates 1 cycle late 
          else
            byteSel <= 9; -- Reset to 1 (back to the start)
          end if;
        end if;
      end if;
    end if;
    end process;
    process(byteSel) 
    begin
    -- Select data based on byteSel value
    case byteSel is
      -- Initialization commands
      when 1  => data <= X"30"; RS <= '0'; -- 4 bit mode select
      when 2  => data <= X"30"; RS <= '0'; -- 4 bit mode select
      when 3  => data <= X"30"; RS <= '0'; -- 4 bit mode select
      when 4  => data <= X"02"; RS <= '0'; -- Initialize 4-bit mode
      when 5  => data <= X"28"; RS <= '0';
      when 6  => data <= X"01"; RS <= '0'; 
      when 7  => data <= X"0E"; RS <= '0'; 
      when 8  => data <= X"06"; RS <= '0'; 
      when 9  => data <= X"80"; RS <= '0'; 

      -- Display messages (reverse order from 127 downto 0)
      when 10 => RS <= '1'; data <= data_in(127 downto 120);
      when 11 => RS <= '1'; data <= data_in(119 downto 112);
      when 12 => RS <= '1'; data <= data_in(111 downto 104);
      when 13 => RS <= '1'; data <= data_in(103 downto 96);
      when 14 => RS <= '1'; data <= data_in(95 downto 88);
      when 15 => RS <= '1'; data <= data_in(87 downto 80);
      when 16 => RS <= '1'; data <= data_in(79 downto 72);
      when 17 => RS <= '1'; data <= data_in(71 downto 64);
      when 18 => RS <= '1'; data <= data_in(63 downto 56);
      when 19 => RS <= '1'; data <= data_in(55 downto 48);
      when 20 => RS <= '1'; data <= data_in(47 downto 40);
      when 21 => RS <= '1'; data <= data_in(39 downto 32);
      when 22 => RS <= '1'; data <= data_in(31 downto 24);
      when 23 => RS <= '1'; data <= data_in(23 downto 16);
      when 24 => RS <= '1'; data <= data_in(15 downto 8);
      when 25 => RS <= '1'; data <= data_in(7 downto 0);
      when 26 => RS <= '0'; data <= X"C0"; -- next line
      when 27 => RS <= '1'; data <= data_in(255 downto 248);
      when 28 => RS <= '1'; data <= data_in(247 downto 240);
      when 29 => RS <= '1'; data <= data_in(239 downto 232);
      when 30 => RS <= '1'; data <= data_in(231 downto 224);
      when 31 => RS <= '1'; data <= data_in(223 downto 216);
      when 32 => RS <= '1'; data <= data_in(215 downto 208);
      when 33 => RS <= '1'; data <= data_in(207 downto 200);
      when 34 => RS <= '1'; data <= data_in(199 downto 192);
      when 35 => RS <= '1'; data <= data_in(191 downto 184);
      when 36 => RS <= '1'; data <= data_in(183 downto 176);
      when 37 => RS <= '1'; data <= data_in(175 downto 168);
      when 38 => RS <= '1'; data <= data_in(167 downto 160);
      when 39 => RS <= '1'; data <= data_in(159 downto 152);
      when 40 => RS <= '1'; data <= data_in(151 downto 144);
      when 41 => RS <= '1'; data <= data_in(143 downto 136);
      when 42 => RS <= '1'; data <= data_in(135 downto 128);
--      when others =>
--        RS     <= '0';
--        LCD_EN <= '0';
--        LCD_DATA<= (others => '0');
--      when others => RS <= '0'; -- Default command

      when others => data <= X"28"; RS <= '0'; -- Default command
    end case;
    end process;
  
end Behavioral;
