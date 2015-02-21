BUS=1
ID=0x68
ATH=0x1b
ATL=0x1c
ARXH=0x1d
ARXL=0x1e
ARYH=0x1f
ARYL=0x20
ARZH=0x21
ARZL=0x22

#need upper case hex stripped of prefix "0x" or bc will not like the input
for VAR in TH TL RXH RXL RYH RYL RZH RZL
do
  CMD="$VAR=\$(i2cget -y $BUS $ID \$A$VAR  b |sed -e "s/^0x//" |tr "a-z" "A-Z")"
  eval $CMD
  eval "echo $VAR = \$$VAR"
done

echo "Temp register: $(echo "ibase=16; $TH$TL" | bc -l) 0x$TH$TL ($(echo "ibase=16; obase=2; $TH$TL" | bc -l))"

echo -n "Temp in Celcius: "
#this takes hex input and evaluates the followin formula in hexadecimal
#temp= 35 + ((raw + 13200) / 280))"
#where raw is the input reading un 2's compliment 
#(to uncompliment the input I take away 0x10000 if input is larger then 0x8000)
echo "ibase=16; input=$TH$TL; if ( input >= 8000 ) { raw=input - 10000;} else { raw=input;}; 23 + ((raw + 3390)/118);" 

echo "X Axis Rotation Register: $(echo "ibase=16; $RXH$RXL" | bc -l) 0x$RXH$RXL ($(echo "ibase=16; obase=2; $RXH$RXL" | bc -l))"
echo -n "X Axis Angula velocity degree/sec: "
echo "ibase=16; input=$RXH$RXL; if ( input >= 8000 ) { raw= input - 10000;} else { raw=input;}; raw / E.177" |bc -l

echo "Y Axis Rotation Register: $(echo "ibase=16; $RYH$RYL" | bc -l) 0x$RYH$RYL ($(echo "ibase=16; obase=2; $RYH$RYL" | bc -l))"
echo -n "Y Axis Angula velocity degree/sec: "
echo "ibase=16; input=$RYH$RYL; if ( input >= 8000 ) { raw= input - 10000;} else { raw=input;}; raw / E.177" |bc -l

echo "Z Axis Rotation Register: $(echo "ibase=16; $RZH$RZL" | bc -l) 0x$RZH$RZL ($(echo "ibase=16; obase=2; $RZH$RZL" | bc -l))"
echo -n "Z Axis Angula velocity degree/sec: "
echo "ibase=16; input=$RZH$RZL; if ( input >= 8000 ) { raw= input - 10000;} else { raw=input;}; raw / E.177" |bc -l
