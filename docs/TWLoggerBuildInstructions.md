**Supplemental Materials**

**Fahlbusch and Harrington 2019**

Tapered Wings Logger (TWLogger) Build Instructions & Circuitry Diagram

**[Equipment]{.underline}**

Soldering iron

Nonconductive table clamps (or prefabricated soldering clamp stand)

Lead-free solder

26-gauge solid core wire

Wire strippers

Electrical tape

Foam paper (e.g., craft supply store)

Glue (e.g., clear liquid super glue)

![A group of items on a table Description automatically
generated](media/image1.jpeg){width="4.180247156605424in"
height="2.3513888888888888in"}

Fig. A1: Sample solder station and equipment. Note the blue rubber dish
is the nonconductive safety pad upon which all soldering takes place.

![](media/image2.jpeg){width="4.178240376202974in"
height="2.3502602799650045in"}

Fig. A2: Example for how to configure clamps to hold circuit boards
while soldering.

**[Components List]{.underline}**

Purchased from [www.adafruit.com](http://www.adafruit.com) (part \#)

1.  Feather M0 Adalogger (\#2796)

2.  Flora LSM303 (\#1247)

3.  Ultimate GPS Breakout (\#746)

4.  Lithium - 3.7v 500mAh (\#1578)

**[Build Instructions]{.underline}**:

This is a 4-part process. Prepare each circuit board separately prior to
combining. Wire lengths are given with the corresponding pin name to
which they need to be soldered.

1.  Accelerometer: Flora LSM303- v1.0

    a.  Solder wires to Flora (i.e., pin name: required wire length)
        (Fig. A3A)

        i.  GRD: 3.5cm

        ii. SCL: 6cm

        iii. SDA: 6cm

        iv. 3V: 3cm, 6cm

    b.  ![](media/image3.tiff){width="3.209722222222222in"
        height="3.0944444444444446in"}![](media/image4.tiff){width="2.5527777777777776in"
        height="3.104861111111111in"}Cut chafe guard from foam sheet and
        glue to back of Flora (Fig. A3B)

Fig. A3. Accelerometer top (A) with wires coming up through pins and
soldered from topside of board; and underside of accelerometer showing
chafe padding (B) (Photo credit: P. Forman).

2.  GPS: Ultimate GPS Breakout -- 66Channel

    c.  Solder wires to GPS (i.e., pin name: required wire length)

        v.  TX: 3.5cm

        vi. RX: 4.0cm

        vii. GRD: 4.5cm

        viii. EN: 4.5cm

        ix. ![](media/image5.tiff){width="2.136111111111111in"
            height="4.553472222222222in"}![](media/image6.tiff){width="4.571527777777778in"
            height="2.9159722222222224in"}VIN: 2.5cm

Fig. A4. Underside of GPS circuit board with wires going through pins to
other side where they are soldered (Photo credit: P. Forman).

Fig. A5. Adding chafe pad to underside of GPS board (Photo credit: P.
Forman).

3.  Connect GPS to Feather board (M0)

    d.  Line up TX, GRD, GRD on the GPS to MO and solder.

    e.  Add small chafe guard to battery area.

4.  Glue Accelerometer to the Feather MO

    f.  Locate the sensor orientation icon on the ACC board and confirm
        it is in line with the text on the Feather M0 (e.g. the "USB"
        text is a good indicator).

    g.  Glue in place.

**APPENDIX B**

Tapered Wings Logger (TWLogger) Software and User Interface

**[Downloads]{.underline}**

1.  SD Formatter

> <https://www.sdcard.org/downloads/formatter_4/>

2.  Arduino

> <https://www.arduino.cc/en/main/software>

3.  R Studio

> <https://www.rstudio.com/products/rstudio/download/>

**[SD Formatter]{.underline}**

1.  Name and format SD card prior to using with TWLogger

**[Arduino]{.underline}**

1.  Install Arduino software

    a.  Follow setup instructions on Adafruit website:
        <https://learn.adafruit.com/adafruit-feather-m0-adalogger/using-with-arduino-ide>

2.  Plug TWLogger into computer

    b.  **Confirm battery is plugged into TWLogger and confirm SD card
        is inserted**

3.  Open TWLogger Arduino software found here:

> <https://github.com/harrington-et-al/TWLogger>

c.  As of 24 June 2019, version to use is
    SensorLoggerV3.2.1\_GPS\_LSM303

<!-- -->

4.  Confirm computer recognizes and can communicate with TWLogger

    d.  ![](media/image7.jpeg){width="5.768055555555556in"
        height="3.092361111111111in"}Tools → Port → \[select COM port
        that lists Feather M0 as connected\] (Fig. B1)

> Fig. B1. Confirm COM port.

e.  If Feather M0 isn't listed, try two options:

    i.  Confirm you properly followed Adafruit setup instructions (step
        1 above)

    ii. ![](media/image8.jpeg){width="5.761111111111111in"
        height="3.0875in"}After confirming proper setup, go to File →
        Examples → Basics → Blink (Fig. B2)

> Fig. B2. Open Blink.

iii. ![](media/image9.jpeg){width="5.788888888888889in"
    height="3.1055555555555556in"}Open Blink and click upload (on the
    toolbar below menu options, click the right-hand arrow that is 2^nd^
    icon from left) (Fig. B3)

> Fig. B3 Upload Blink.

iv. Note: Timing is important in this step. Watch progress report
    (bottom left, orange writing on black area of program screen). After
    it switches from "Compiling Sketch" to "Uploading" quickly double
    click reset button on TWLogger.

v.  Now check if Feather M0 is listed as available on a COM port

<!-- -->

5.  After correct COM port is selected, press upload (on the toolbar
    below menu options, click the right-hand arrow that is 2^nd^ icon
    from left) to upload TWLogger software.

6.  ![](media/image10.jpeg){width="5.651388888888889in"
    height="3.0416666666666665in"}Immediately after progress bar says
    "Done Uploading" (bottom left, orange writing on black area of
    program screen), go to Tools → Serial Monitor (Fig. B4)

> Fig. B4. Open serial monitor programming menu.

f.  Complete programming prompts by entering the number into the text
    bar

g.  for the parameter you would like to define, then press enter. When
    prompted, enter preferred settings (Table B1; Fig. B5).

+-----------------------+-----------------------+-----------------------+
| Settings Prompt       | Parameter Options     | Description           |
+=======================+=======================+=======================+
| Enter Tag Number      | 1-99                  | User enters tag       |
|                       |                       | number which sets     |
|                       |                       | initialization file   |
|                       |                       | name                  |
+-----------------------+-----------------------+-----------------------+
| Set Date and Time     | YYYY-m-d H:M:S        | User syncs the date   |
| (GMT)                 |                       | and time to a GPS     |
+-----------------------+-----------------------+-----------------------+
| Display Time          | \-                    | Displays the clock to |
|                       |                       | confirm sync with     |
|                       |                       | external source       |
+-----------------------+-----------------------+-----------------------+
| Set Sampling Rate     | 5 Hz, 10 Hz, 25 Hz,   | User sets the         |
|                       | 40 Hz, 50 Hz          | sampling rate         |
+-----------------------+-----------------------+-----------------------+
| Set Local Time Zone   | +/- GMT, hour 0-12    | User defines local    |
| Offset                |                       | offset from GMT       |
+-----------------------+-----------------------+-----------------------+
| Check GPS Fix         | \-                    | Displays scrolling    |
|                       |                       | status of GPS fix     |
+-----------------------+-----------------------+-----------------------+
| Set GPS Parameters    | Sampling Rate (s) -   | User selects interval |
|                       | default is 120        | between GPS data      |
|                       | (1-600)               | acquisition           |
|                       |                       |                       |
|                       | Timeout (s) - default | User selects interval |
|                       | is 60 (20-90)         | to wait for a fix     |
|                       |                       | before sleeping       |
|                       | Read Delay (s) -      |                       |
|                       | default is 8 (1-30)   | User selects interval |
|                       |                       | to wait before trying |
|                       | Samples per interval  | to log (allows for a  |
|                       | - default is 1 (1-30) | fix)                  |
|                       |                       |                       |
|                       | Smart Delay (ms) -    | Interval to spend     |
|                       | default is 10 (1-20)  | encoding GPS data     |
|                       |                       | between other         |
|                       |                       | instructions          |
+-----------------------+-----------------------+-----------------------+
| Start Logging         | \-                    | User selects start    |
|                       |                       | logging to initialize |
|                       |                       | the tag               |
+-----------------------+-----------------------+-----------------------+

![](media/image11.jpeg){width="5.660416666666666in"
height="3.0347222222222223in"}Table B1. User-defined parameters
available in the TWLogger setup state.

Fig. B5. Serial monitor parameter options.

vi. Troubleshooting: If Serial Monitor screen is blank, press spacebar
    and then press enter. The prompts should appear.

vii. Date: This will default to the date the software was uploaded.

viii. Time: Always program in UTC.

ix. GPS: To check GPS is able to properly establish satellite
    connections,

<!-- -->

h.  To begin logging, enter parameter number 9 in the serial monitor
    text bar and press enter. Logger will begin logging.

<!-- -->

7.  Unplug TWLogger \-\-- TWLogger is logging!

    i.  A red LED pulse will confirm the device is logging properly. The
        microprocessor will blink at top of each minute and the GPS will
        blink at 1-Hz while it is attempting a fix.

    j.  If the microprocessor LED is flashing rapidly, it means there is
        an error. The most likely cause the inability to read or write
        to the SD card.

**[R Studio]{.underline}**

1.  tagtools packages

> http://www.animaltags.org/doku.php?id=tagwiki:info:people

2.  See
    [http://github.com/Harrington-et-al/TWLogger](http://github.com/jamesfahlbusch/TWLogger/AnalysisToolkit)
    for documented analysis toolkit.
