
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.numeric_std.all;

entity LCD_I2C_user_logic is
  generic (
    input_clk : integer := 125_000_000; --input clock speed from user logic in Hz
    bus_clk   : integer := 50_000); --speed the i2c bus (scl) will run at in Hz
  port (
    clk   : in std_logic;
    reset : in std_logic;
    data_in : in std_logic_vector(255 downto 0);
    scl : inout std_logic;
    sda : inout std_logic
  );
end LCD_I2C_user_logic;

-- -----------------------------------------------------------------------------------------------------------------------------------

architecture user_logic of LCD_I2C_user_logic is

  component i2c_master is
    generic (
      input_clk : integer := 125_000_000; --input clock speed from user logic in Hz
      bus_clk   : integer := 50_000); --speed the i2c bus (scl) will run at in Hz
    -- ADC runs at 400,000, lcd at 100k max 
    port (
      clk       : in std_logic; --system clock
      reset_n   : in std_logic; --active low reset
      ena       : in std_logic; --latch in command
      addr      : in std_logic_vector(6 downto 0); --address of target slave
      rw        : in std_logic; --'0' is write, '1' is read
      data_wr   : in std_logic_vector(7 downto 0); --data to write to slave
      busy      : out std_logic; --indicates transaction in progress
      data_rd   : out std_logic_vector(7 downto 0); --data read from slave
      ack_error : buffer std_logic; --flag if improper acknowledge from slave
      sda       : inout std_logic; --serial data output of i2c bus
      scl       : inout std_logic); --serial clock output of i2c bus
  end component;
  component LCDDataSelect is
    port (
      clk      : in std_logic;
      reset    : in std_logic;
      busy     : in std_logic;
      data_in  : in std_logic_vector(255 downto 0);
--      MODE     : in std_logic_vector(2 downto 0)   := "001";
      data_out : out std_logic_vector(7 downto 0)
    );
  end component;
  -- -----------------------------------------------------------------------------------------------------------------------------------

  signal LCD_Data : std_logic_vector(7 downto 0) := (others => '0');

  signal cont        : unsigned(27 downto 0)        := X"0003FFF";
  signal slave_addr  : std_logic_vector(6 downto 0) := "0100111"; -- 0x27 in 7-bit
  signal i2c_addr    : std_logic_vector(6 downto 0);
  signal i2c_rw      : std_logic                    := '0';
  signal i2c_ena     : std_logic                    := '0';
  signal i2c_data_wr : std_logic_vector(7 downto 0) := (others => '0');
  type state_type is (start, write);
  signal state   : state_type := start;
  signal rst     : std_logic  := '0';
  signal reset_M : std_logic;
  signal reset_D : std_logic := '0';
  signal busy    : std_logic;
  --signal data_in : std_logic_vector(127 downto 0) := X"48616E675F5F5F5F4D616E2020202020";
  -- -----------------------------------------------------------------------------------------------------------------------------------
begin
  reset_M <= not reset or not rst; -- active low
  reset_D <= not reset_M; -- active high
  i2c_rw  <= '0';

  inst_i2cMaster : i2c_master
  generic map(
    input_clk => 125_000_000, --input clock speed from user logic in Hz
    bus_clk   => 50_000) --speed the i2c bus (scl) will run at in Hz
  port map
  (
    clk       => clk, --system clock
    reset_n   => reset_M, --active low reset
    ena       => i2c_ena, --latch in command
    addr      => i2c_addr, --address of target slave
    rw        => i2c_rw, --'0' is write, '1' is read (I am writing data ABCD)
    data_wr   => i2c_data_wr, --data to write to slave
    busy      => busy, --indicates transaction in progress
    data_rd   => open, --data read from slave (e.g. a sensor)
    ack_error => open, --flag if improper acknowledge from slave
    sda       => sda, --serial data output of i2c bus
    scl       => scl
  );
  inst_dataSelect : LCDDataSelect
  port map
  (
    clk      => clk,
    reset    => reset_D,
    busy     => busy,
    data_in  => data_in,
    data_out => LCD_Data
  );

  process (clk)
  begin
    if reset = '1' then
      rst         <= '1';
      cont        <= X"0003FFF";
      i2c_addr    <= slave_addr;
      i2c_data_wr <= (others => '0');
      i2c_ena     <= '0';
      state       <= start;
    elsif rising_edge(clk) then
      case state is
        when start =>
          if (cont /= X"0000000") then
            cont  <= cont - 1;
            rst   <= '1';
            state <= start;
          else
            rst      <= '0';
            i2c_ena  <= '1';
            i2c_addr <= slave_addr;
            state    <= write;
          end if;
        when write =>
          i2c_data_wr <= LCD_Data;
          state       <= write;
        when others =>
          state <= start;

      end case;
    end if;
  end process;

end user_logic;
