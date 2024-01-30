; *****************************************************************
; * This stationery serves as the framework for a                 *
; * user application (single file, absolute assembly application) *
; * For a more comprehensive program that                         *
; * demonstrates the more advanced functionality of this          *
; * processor, please see the demonstration applications          *
; * located in the examples subdirectory of the                   *
; * Freescale CodeWarrior for the HC12 Program directory          *
; *****************************************************************

              XDEF Entry, _Startup ;
              ABSENTRY Entry
              
              INCLUDE "derivative.inc"

; Insert here your data definition
;***************************************************************************************************
;*                                      FINAL PROJECT                                              *                         
;***************************************************************************************************

; equates section
;***************************************************************************************************
LCD_DAT         EQU   PORTB                   ; LCD data port, bits - PB7,...,PB0
LCD_CNTR        EQU   PTJ                     ; LCD control port, bits - PE7(RS),PE4(E)
LCD_E           EQU   $80                     ; LCD E-signal pin
LCD_RS          EQU   $40                     ; LCD RS-signal pin
TURN_T          EQU   8                       ; Turn timer for 8/23rd of a second
START           EQU   0                       ; Start State
FWD             EQU   1                       ; Forward State
ALL_STOP        EQU   2                       ; All Stop State
LEFT_TRN        EQU   3                       ; Left Turn State
RIGHT_TRN       EQU   4                       ; Right Turn State
REV_TRN         EQU   5                       ; Reverse Turn State
LEFT_CENTERED   EQU   6                       ; Left Centering State
RIGHT_CENTERED  EQU   7                       ; Right Centering State

; variable section
;***************************************************************************************************
                          ORG   $3800
BASE_LINE                 FCB   $9D           ; Baseline threshold determined through guider
BASE_BOW                  FCB   $CA           ; Baseline threshold determined through guider
BASE_MID                  FCB   $CA           ; Baseline threshold determined through guider
BASE_PORT                 FCB   $BF           ; Baseline threshold determined through guider
BASE_STBD                 FCB   $CC           ; Baseline threshold determined through guider

LINE_ACC_DEVIATION        FCB   $18           ; Adding a deviation to account for ambient lighting and other factors 
BOW_ACC_DEVIATION         FCB   $30           ;   "
PORT_ACC_DEVIATION        FCB   $30           ;   "          
MID_ACC_DEVIATION         FCB   $20           ;   "
STARBOARD_ACC_DEVIATION   FCB   $15           ;   "

TEMP          RMB   1                       ; Dummy variable for use throughout the program

SENSOR_LINE   FCB   $01                     ; Memory Address for sensor readings
SENSOR_BOW    FCB   $23                     ;     "
SENSOR_PORT   FCB   $45                     ;     "
SENSOR_MID    FCB   $67                     ;     "
SENSOR_STBD   FCB   $89                     ;     "
SENSOR_NUM    RMB   1                       ; Variable to store the sensor number being evaluated

              ORG   $3850                   ; Where our TOF counter register lives
TOF_COUNTER   dc.b  0                       ; The timer, incremented at 23Hz
CRNT_STATE    dc.b  2                       ; Current state register
T_TURN        ds.b  1                       ; Variable to hold timer alarm
TEN_THOUS     ds.b  1                       ; 10,000 digit
THOUSANDS     ds.b  1                       ;  1,000 digit
HUNDREDS      ds.b  1                       ;    100 digit
TENS          ds.b  1                       ;     10 digit
UNITS         ds.b  1                       ;      1 digit
NO_BLANK      ds.b  1                       ; Used in 'leading zero' blanking by BCD2ASC
BCD_SPARE     RMB   2                       ; Extra space for decimal point and string terminator

; code section
;***************************************************************************************************
              ORG   $4000
Entry:                                                                       
_Startup: 
              LDS   #$4000                 ; Initialize the stack pointer
              CLI                          ; Enable interrupts
              
              JSR   INIT                   ; Initialize the sensors
              
              BSET  DDRA,%00000011         ; STAR_DIR, PORT_DIR                        
              BSET  DDRT,%00110000         ; STAR_SPEED, PORT_SPEED
                                  
              JSR   initAD                 ; Initialize ATD converter
                                
              JSR   initLCD                ; Initialize the LCD                        
              JSR   clrLCD                 ; Clear LCD & home cursor
                                 
              LDX   #msg1                  ; Display msg1                              
              JSR   putsLCD                ;       "     
                                            
              LDAA  #$C0                   ; Move LCD cursor to the 2nd row           
              JSR   cmd2LCD                ;                                           
              LDX   #msg2                  ; Display msg2                              
              JSR   putsLCD                ;       "
                    
              JSR   ENABLE_TOF             ; Jump to TOF initialization
MAIN        
              JSR   G_LEDS_ON              ; Enable the guider LEDs   
              JSR   READ_SENSORS           ; Read the 5 guider sensors
              JSR   G_LEDS_OFF             ; Disable the guider LEDs                   
              JSR   UPDT_DISPL             ; Update the Batter Voltage and Current State
              LDAA  CRNT_STATE             ; Load the Current State into Accumulator A
              JSR   DISPATCHER             ; Jump to the Dispatcher
              BRA   MAIN                   ; Loop back to 'MAIN' label

; data section
;***************************************************************************************************
msg1          dc.b  "Battery volt ",0
msg2          dc.b  "State",0
tab           dc.b  "Start  ",0
              dc.b  "Fwd    ",0
              dc.b  "All_Stp",0
              dc.b  "Lft_Trn",0
              dc.b  "RghtTrn",0
              dc.b  "Rev_Trn",0
              dc.b  "LftAdj ",0     
              dc.b  "RghtAdj",0  

; subroutine section
;***************************************************************************************************
DISPATCHER        CMPA  #START                                ; If the robot's state is START
                  BNE   NOT_START                             ; If not, jump to the NOT_START
                  JSR   START_ST                              ; Otherwise, go to the START_ST subroutine
                  BRA   DISPATCH_EXIT                                         

NOT_START         CMPA  #FWD                                  ; If the robot's state is FORWARD
                  BNE   NOT_FORWARD                           ; If not, jump to the NOT_FORWARD
                  JSR   FWD_ST                                ; Otherwise, go to the FWD_ST subroutine
                  BRA   DISPATCH_EXIT

NOT_FORWARD       CMPA  #ALL_STOP                             ; If the robot's state is ALL_STOP
                  BNE   NOT_ALL_STOP                          ; If not, jump to the NOT_ALL_STOP 
                  JSR   ALL_STOP_ST                           ; Otherwise, go to the ALL_STOP_ST subroutine
                  BRA   DISPATCH_EXIT     

NOT_ALL_STOP      CMPA  #LEFT_TRN                             ; If the robot's state is LEFT_TRN
                  BNE   NOT_LEFT_TURN                         ; If not, jump to the NOT_LEFT_TURN
                  JSR   LEFT                                  ; Otherwise, go to the LEFT subroutine
                  BRA   DISPATCH_EXIT    

NOT_LEFT_TURN     CMPA  #RIGHT_TRN                            ; If the robot's state is RIGHT_TRN
                  BNE   NOT_RIGHT_TURN                        ; If not, jump to the NOT_RIGHT_TURN
                  JSR   RIGHT                                 ; Otherwise, go to the RIGHT subroutine        
                  BRA   DISPATCH_EXIT                                                  

NOT_RIGHT_TURN    CMPA  #REV_TRN                              ; If the robot's state is REV_TRN
                  BNE   NOT_REV_TURN                          ; If not, jump to the NOT_REV_TURN
                  JSR   REV_TRN_ST                            ; Otherwise, go to the REV_TRN_ST subroutine
                  BRA   DISPATCH_EXIT                                          

NOT_REV_TURN      CMPA  #LEFT_CENTERED                        ; If the robot's state is LEFT_CENTERED
                  BNE   NOT_LEFT_CENTERED                     ; If not, jump to the LEFT_CENTERED
                  JSR   LEFT_CENTERED_COMP                    ; Otherwise, go to the LEFT_CENTERED_COMP subroutine
                  BRA   DISPATCH_EXIT

NOT_LEFT_CENTERED CMPA  #RIGHT_CENTERED                       ; If the robot's state is RIGHT_CENTERED
                  JSR   RIGHT_CENTERED_COMP                   ; jump to the RIGHT_CENTERED_COMP
                  BRA   DISPATCH_EXIT                         ; Otherwise, we have reached an invalid state

DISPATCH_EXIT     RTS

;***************************************************************************************************
START_ST          BRCLR   PORTAD0, %00000100,START_EXIT       ; If FWD_BUMP is not pressed then exit                             
                  JSR     INIT_FWD                            ; Otherwise initiate forward                                   
                  MOVB    #FWD, CRNT_STATE                    ; Update current state to forward

START_EXIT        RTS                                                                                                                                  

;***************************************************************************************************
FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP      ; If FWD_BUMP is not pressed then branch                           
                  MOVB    #REV_TRN, CRNT_STATE           ; Otherwise change current state to REV_TRN                                 
                  JSR     UPDT_DISPL                     ; Update the display with the current state                                
                  JSR     INIT_REV                       ; Initiate the Reverse Subroutine                                    
                  LDY     #10000                         ; Load Y with 10000                                     
                  JSR     del_50us                       ; 10000x50us = 0.5s delay => 0.5s of Reverse                                   
                  JSR     INIT_RIGHT                     ; Initialize the Right Turn Subroutine                                    
                  LDY     #6000                          ; Load Y with 6000                                    
                  JSR     del_50us                       ; 6000x50us = 0.3s => 0.3s of Right Turn                                   
                  RTS                                    ; Return to Dispatch -> Main -> Dispatch -> REV_TRN                           

NO_FWD_BUMP       BRSET   PORTAD0, $04, NO_FWD_REAR_BUMP ; If FWD_BUMP is not pressed then branch
                  MOVB    #ALL_STOP, CRNT_STATE          ; Otherwise change current state to ALL_STOP                    
                  JSR     INIT_ALL_STOP                  ; Initiate the All_STOP Subroutine 
                  RTS 

NO_FWD_REAR_BUMP  LDAA    SENSOR_BOW                     ; If neither bumper is pressed load A with Bow Sensor Value                                         
                  ADDA    BOW_ACC_DEVIATION              ; Add the Bow Sensor deviation to that Bow Sensor value                                                 
                  CMPA    BASE_BOW                       ; Compare this summed up value to the baseline value for Bow                                         
                  BPL     NOT_CENTERED                   ; If it is larger than or equal to the baseline, branch                                             
                  LDAA    SENSOR_MID                     ; load A with Mid Sensor Value                                        
                  ADDA    MID_ACC_DEVIATION              ; Add the Mid Sensor deviation to that Mid Sensor value                                                  
                  CMPA    BASE_MID                       ; Compare this summed up value to the baseline value for Mid                                         
                  BPL     NOT_CENTERED                   ; If it is larger than or equal to the baseline, branch                                             
                  LDAA    SENSOR_LINE                    ; load A with Line Sensor Value (E-F)                                        
                  ADDA    LINE_ACC_DEVIATION             ; Add the Line Sensor deviation to that Line Sensor value                                                  
                  CMPA    BASE_LINE                      ; Compare this summed up value to the baseline value for Line                                         
                  LBPL    CHECK_RIGHT_CENT               ; If it is larger than or equal to the baseline, long branch                                          
                  LDAA    SENSOR_LINE                    ; load A with Line Sensor Value (E-F)                                        
                  SUBA    LINE_ACC_DEVIATION             ; Subtract the Line Sensor deviation to that Line Sensor value                                                   
                  CMPA    BASE_LINE                      ; Compare this difference to the baseline value for Line                                        
                  BMI     CHECK_LEFT_CENT                ; If it is smaller than or equal to the baseline, branch
 
;***************************************************************************************************                                                                  
NOT_CENTERED      LDAA    SENSOR_PORT                    ; Load A with Port Sensor Value                                        
                  ADDA    PORT_ACC_DEVIATION             ; Add the Port Deviation to it                                                  
                  CMPA    BASE_PORT                      ; Compare it to the Baseline Port value                                        
                  BPL     SLIGHT_LEFT_TRN                ; If it is larger then branch to Slight_Left_Turn                                        
                  BMI     NO_PORT                        ; If it is smaller then branch to No_Port                                     

NO_PORT           LDAA    SENSOR_BOW                     ; If port was smaller than baseline, load A with Bow value                                        
                  ADDA    BOW_ACC_DEVIATION              ; Add Bow Deviation to it                                                   
                  CMPA    BASE_BOW                       ; Compare it to the Baseline Bow value                                         
                  BPL     RETURN_MAIN                    ; If it is larger then branch back to dispatcher                                                
                  BMI     NO_BOW                         ; If it is less then branch to No_Bow                                     

NO_BOW            LDAA    SENSOR_STBD                    ; If bow was smaller than baseline, load A with Starboard value                                         
                  ADDA    STARBOARD_ACC_DEVIATION        ; Add Starboard Deviation to it                                                       
                  CMPA    BASE_STBD                      ; Compare it to the Baseline Starboard value                                         
                  BPL     SLIGHT_RIGHT_TRN               ; If it is larger then branch to Slight_Right_Turn                                          
                  BMI     RETURN_MAIN                    ; If it is smaller then return to Dispatcher
;***************************************************************************************************                                                                            
LEFT                LDAA    SENSOR_BOW                   ; Load A with Bow Sensor Value                                           
                    ADDA    BOW_ACC_DEVIATION            ; Add the Bow deviation to it                                                    
                    CMPA    BASE_BOW                     ; Compare it to the Bow Baseline                                          
                    BPL     LEFT_CENTERED_COMP           ; If it is larger then branch to Left_Centered_Comp                                            
                    BMI     RETURN_MAIN                  ; If it is less then branch back to Dispatch which will come back here

LEFT_CENTERED_COMP  MOVB    #FWD, CRNT_STATE             ; If left center is complete, change the state to FWD                                           
                    JSR     INIT_FWD                     ; Initiate Forward                                           
                    RTS                                                                                       

SLIGHT_LEFT_TRN     LDY     #7000                        ; Load Y with 7000                                         
                    JSR     del_50us                     ; 7000x50us =0.35s delay                                           
                    JSR     INIT_LEFT                    ; Then initiate left turn                                           
                    MOVB    #LEFT_TRN, CRNT_STATE        ; Change current state to left turn                                          
                    LDY     #7000                        ; Load Y with 7000                                           
                    JSR     del_50us                     ; 7000x50us = 0.35s delay => 0.35s of left turn                                           
                    RTS                                                                    

CHECK_LEFT_CENT     JSR     INIT_LEFT                    ; Initiate left turn                                           
                    MOVB    #LEFT_CENTERED, CRNT_STATE   ; Change current state to Left_Centered                                              
                    RTS

;*************************************************************************************************** 
RIGHT               LDAA    SENSOR_BOW                   ; Load A with the Bow Sensor                                           
                    ADDA    BOW_ACC_DEVIATION            ; Add the Bow deviation to the sensor value                                                    
                    CMPA    BASE_BOW                     ; Compare it to the baseline bow value                                          
                    BPL     RIGHT_CENTERED_COMP          ; If it is larger then branch                                             
                    BMI     RETURN_MAIN                  ; If it is less then branch back to the dispatcher

RIGHT_CENTERED_COMP MOVB    #FWD, CRNT_STATE             ; Bow sensor was larger than baseline, so change current state to forward                                           
                    JSR     INIT_FWD                     ; Initiate the forward state                                           
                    RTS    
                  
SLIGHT_RIGHT_TRN    LDY     #6000                        ; Load Y with 6000                                          
                    JSR     del_50us                     ; 6000x50 = 0.3s                                           
                    JSR     INIT_RIGHT                   ; Initialize right subroutine                                           
                    MOVB    #RIGHT_TRN, CRNT_STATE       ; Change the current state to Right Turn                                          
                    LDY     #6000                        ; Load Y with 6000                                           
                    JSR     del_50us                     ; 6000x50 = 0.3s  => 0.3s of turning right                                           
                    RTS                                                                   

CHECK_RIGHT_CENT    JSR     INIT_RIGHT                   ; Initiate Right turn                                           
                    MOVB    #RIGHT_CENTERED, CRNT_STATE  ; Change current state to Right_Centered                                               
                    BRA     RETURN_MAIN                  ; Return to Dispatch                                                                                                                                       

RETURN_MAIN         RTS

;***************************************************************************************************
REV_TRN_ST        LDAA    SENSOR_BOW                     ; Load A with the Bow Sensor                                         
                  ADDA    BOW_ACC_DEVIATION              ; Add the Bow deviation to the sensor value                                                    
                  CMPA    BASE_BOW                       ; Compare it to the baseline bow value                                           
                  BMI     NO_START_BUMP                  ; If it is less then branch back to caller/ keep looping                                                  
                  JSR     INIT_LEFT                      ; If it is greater then initiate left turn                                         
                  MOVB    #FWD, CRNT_STATE               ; Change current state to Forward                                         
                  JSR     INIT_FWD                       ; Initiate forward state                                         
                  RTS                                                                   

ALL_STOP_ST       BRSET   PORTAD0, %00000100, NO_START_BUMP ; If front bumper is not pressed branch to No_Start_Bumper                                       
                  MOVB    #START, CRNT_STATE                ; Else change current state to Start and go back to dispatcher                                     

NO_START_BUMP     RTS                                                                             

; Initialization Subroutines
;***************************************************************************************************
INIT_FWD          BCLR    PORTA, %00000011          ; Set FWD direction for both motors
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  RTS 

INIT_REV          BSET PORTA,%00000011              ; Set REV direction for both motors
                  BSET PTT,%00110000                ; Turn on the drive motors
                  RTS
                  
INIT_RIGHT        BSET    PORTA,%00000010          
                  BCLR    PORTA,%00000001           
                  RTS

INIT_LEFT         BSET    PORTA,%00000001         
                  BCLR    PORTA,%00000010          
                  RTS

INIT_ALL_STOP     BCLR    PTT, %00110000            ; Turn off the drive motors
                  RTS

;***************************************************************************************************
;       Initialize Sensors
INIT              BCLR   DDRAD,$FF ; Make PORTAD an input (DDRAD @ $0272)
                  BSET   DDRA,$FF  ; Make PORTA an output (DDRA @ $0002)
                  BSET   DDRB,$FF  ; Make PORTB an output (DDRB @ $0003)
                  BSET   DDRJ,$C0  ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                  RTS

;***************************************************************************************************
; -------------------------------------------------------------------------------------------------      
;                                   Guider LEDs ON                                                 |
; This routine enables the guider LEDs so that readings of the sensor                              |
; correspond to the illuminated situation.                                                         |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |

G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                                 |
                  RTS                                                                             ;|

; -------------------------------------------------------------------------------------------------      
;                                   Guider LEDs OFF                                                |
; This routine disables the guider LEDs. Readings of the sensor                                    |
; correspond to the ambient lighting situation.                                                    |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |
G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                               |
                  RTS                                                                             ;|    

; -------------------------------------------------------------------------------------------------      
;                               Read Sensors
READ_SENSORS      CLR   SENSOR_NUM     ; Select sensor number 0
                  LDX   #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP      LDAA  SENSOR_NUM     ; Select the correct sensor input
                  JSR   SELECT_SENSOR  ; on the hardware
                  LDY   #400           ; 20 ms delay to allow the
                  JSR   del_50us       ; sensor to stabilize
                  LDAA  #%10000001     ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L        ; A/D conversion is complete in ATDDR0L
                  STAA  0,X            ; so copy it to the sensor register
                  CPX   #SENSOR_STBD   ; If this is the last reading
                  BEQ   RS_EXIT        ; Then exit
                  INC   SENSOR_NUM     ; Else, increment the sensor number
                  INX                  ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP   ; and do it again

RS_EXIT           RTS

; -------------------------------------------------------------------------------------------------      
;                               Select Sensor
; -------------------------------------------------------------------------------------------------      
SELECT_SENSOR     PSHA                ; Save the sensor number for the moment
                  LDAA PORTA          ; Clear the sensor selection bits to zeros
                  ANDA #%11100011
                  STAA TEMP           ; and save it into TEMP
                  PULA                ; Get the sensor number
                  ASLA                ; Shift the selection number left, twice
                  ASLA 
                  ANDA #%00011100     ; Clear irrelevant bit positions
                  ORAA TEMP           ; OR it into the sensor bit positions
                  STAA PORTA          ; Update the hardware
                  RTS

;***************************************************************************************************
;*                      Update Display (Battery Voltage + Current State)                           *
;***************************************************************************************************
UPDT_DISPL        MOVB    #$90,ATDCTL5    ; R-just., uns., sing. conv., mult., ch=0, start
                  BRCLR   ATDSTAT0,$80,*  ; Wait until the conver. seq. is complete
                  LDAA    ATDDR0L         ; Load the ch0 result - battery volt - into A
                  LDAB    #39             ;AccB = 39
                  MUL                     ;AccD = 1st result x 39
                  ADDD    #600            ;AccD = 1st result x 39 + 600
                  JSR     int2BCD
                  JSR     BCD2ASC
                  LDAA    #$8D            ;move LCD cursor to the 1st row, end of msg1
                  JSR     cmd2LCD
                  LDAA    TEN_THOUS       ;output the TEN_THOUS ASCII character
                  JSR     putcLCD 
                  LDAA    THOUSANDS       ;output the THOUSANDS character
                  JSR     putcLCD
                  LDAA    #'.'            ; add the decimal place
                  JSR     putcLCD         ; put the dot into LCD
                  LDAA    HUNDREDS        ;output the HUNDREDS ASCII character
                  JSR     putcLCD         ;same for THOUSANDS,  .  and HUNDREDS
                  LDAA    #$C7            ; Move LCD cursor to the 2nd row, end of msg2
                  JSR     cmd2LCD         ;
                  LDAB    CRNT_STATE      ; Display current state
                  LSLB                    ; "
                  LSLB                    ; "
                  LSLB
                  LDX     #tab            ; "
                  ABX                     ; "
                  JSR     putsLCD         ; "
                  RTS

;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable TCNT
                  STAA    TFLG2           ; Clear TOF
                  LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS

TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000      ; Clear
                  STAA    TFLG2           ; TOF
                  RTI

; utility subroutines
;***************************************************************************************************
initLCD:          BSET    DDRB,%11111111  ; configure pins PS7,PS6,PS5,PS4 for output
                  BSET    DDRJ,%11000000  ; configure pins PE7,PE4 for output
                  LDY     #2000
                  JSR     del_50us
                  LDAA    #$28
                  JSR     cmd2LCD
                  LDAA    #$0C
                  JSR     cmd2LCD
                  LDAA    #$06
                  JSR     cmd2LCD
                  RTS

;***************************************************************************************************
clrLCD:           LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us
                  RTS

;***************************************************************************************************
del_50us          PSHX                   ; (2 E-clk) Protect the X register
eloop             LDX   #300             ; (2 E-clk) Initialize the inner loop counter
iloop             NOP                    ; (1 E-clk) No operation
                  DBNE X,iloop           ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop           ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX                   ; (3 E-clk) Restore the X register
                  RTS                    ; (5 E-clk) Else return

;***************************************************************************************************
cmd2LCD:          BCLR  LCD_CNTR, LCD_RS ; select the LCD instruction
                  JSR   dataMov          ; send data to IR
                  RTS

;***************************************************************************************************
putsLCD:          LDAA  1,X+             ; get one character from  string
                  BEQ   donePS           ; get NULL character
                  JSR   putcLCD
                  BRA   putsLCD

donePS            RTS

;***************************************************************************************************
putcLCD:          BSET  LCD_CNTR, LCD_RS  ; select the LCD data register (DR)c
                  JSR   dataMov           ; send data to DR
                  RTS

;***************************************************************************************************
dataMov:          BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the upper 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LSLA                    ; match the lower 4 bits with LCD data pins
                  LSLA                    ; ""
                  LSLA                    ; ""
                  LSLA                    ; ""
                  BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the lower 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LDY   #1                ; adding this delay allows
                  JSR   del_50us          ; completion of most instructions
                  RTS

;***************************************************************************************************
initAD            MOVB  #$C0,ATDCTL2      ;power up AD, select fast flag clear
                  JSR   del_50us          ;wait for 50 us
                  MOVB  #$00,ATDCTL3      ;8 conversions in a sequence
                  MOVB  #$85,ATDCTL4      ;res=8, conv-clks=2, prescal=12
                  BSET  ATDDIEN,$0C       ;configure pins AN03,AN02 as digital inputs
                  RTS

;***************************************************************************************************
int2BCD           XGDX                    ;Save the binary number into .X
                  LDAA #0                 ;Clear the BCD_BUFFER
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  CPX #0                  ; Check for a zero input
                  BEQ CON_EXIT            ; and if so, exit
                  XGDX                    ; Not zero, get the binary number back to .D as dividend
                  LDX #10                 ; Setup 10 (Decimal!) as the divisor
                  IDIV                    ; Divide Quotient is now in .X, remainder in .D
                  STAB UNITS              ; Store remainder
                  CPX #0                  ; If quotient is zero,
                  BEQ CON_EXIT            ; then exit
                  XGDX                    ; else swap first quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TEN_THOUS

CON_EXIT          RTS                     ; Were done the conversion

LCD_POS_CRSR      ORAA #%10000000         ; Set the high bit of the control word
                  JSR cmd2LCD             ; and set the cursor address
                  RTS

;***************************************************************************************************
HEX_TABLE     FCC   '0123456789ABCDEF'      ; Table for converting values
BIN2ASC               PSHA                ; Save a copy of the input number
                      TAB            
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the LSnibble
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                
                      LDAA 0,X            ; Get the LSnibble character
                      PULB                ; Retrieve the input number into ACCB
                      PSHA                ; and push the LSnibble character in its place
                      RORB                ; Move the upper nibble of the input number
                      RORB                ;  into the lower nibble position.
                      RORB
                      RORB 
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the MSnibble 
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                                                               
                      LDAA 0,X            ; Get the MSnibble character into ACCA
                      PULB                ; Retrieve the LSnibble character into ACCB
                      RTS

;***************************************************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ?NO_BLANK? starts cleared and is set once a non-zero
;* digit has been detected.
;* The ?units? digit is never blanked, even if it and all the
;* preceding digits are zero.

BCD2ASC           LDAA    #0            ; Initialize the blanking flag
                  STAA    NO_BLANK

C_TTHOU           LDAA    TEN_THOUS     ; Check... (6 KB left)
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK1

ISBLANK1          LDAA    #' '          ; It s blank
                  STAA    TEN_THOUS     ; so store a space
                  BRA     C_THOU        ; and check the  thousands  digit

NOT_BLANK1        LDAA    TEN_THOUS     ; Get the  ten_thousands  digit
                  ORAA    #$30          ; Convert to ascii
                  STAA    TEN_THOUS
                  LDAA    #$1           ; Signal that we have seen a  non-blank  digit
                  STAA    NO_BLANK

C_THOU            LDAA    THOUSANDS     ; Check the thousands digit for blankness
                  ORAA    NO_BLANK      ; If it s blank and  no-blank  is still zero
                  BNE     NOT_BLANK2

ISBLANK2          LDAA    #' '          ; Thousands digit is blank
                  STAA    THOUSANDS     ; so store a space
                  BRA     C_HUNS        ; and check the hundreds digit

NOT_BLANK2        LDAA    THOUSANDS     ; (similar to  ten_thousands  case)
                  ORAA    #$30
                  STAA    THOUSANDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_HUNS            LDAA    HUNDREDS      ; Check the hundreds digit for blankness
                  ORAA    NO_BLANK      ; If it s blank and  no-blank  is still zero
                  BNE     NOT_BLANK3

ISBLANK3          LDAA    #' '          ; Hundreds digit is blank
                  STAA    HUNDREDS       ; so store a space
                  BRA     C_TENS          ; and check the tens digit

NOT_BLANK3        LDAA    HUNDREDS          ; (similar to  ten_thousands  case)
                  ORAA    #$30
                  STAA    HUNDREDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_TENS            LDAA    TENS          ; Check the tens digit for blankness
                  ORAA    NO_BLANK      ; If it s blank and  no-blank  is still zero
                  BNE     NOT_BLANK4

ISBLANK4          LDAA    #' '          ; Tens digit is blank
                  STAA    TENS          ; so store a space
                  BRA     C_UNITS       ; and check the units digit

NOT_BLANK4        LDAA    TENS          ; (similar to  ten_thousands  case)
                  ORAA    #$30
                  STAA    TENS

C_UNITS           LDAA    UNITS         ; No blank check necessary, convert to ascii.
                  ORAA    #$30
                  STAA    UNITS
                  RTS                 

;***************************************************************************************************
;*                                Interrupt Vectors                                                *
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry ; Reset Vector
                  ORG     $FFDE
                  DC.W    TOF_ISR ; Timer Overflow Interrupt Vector


