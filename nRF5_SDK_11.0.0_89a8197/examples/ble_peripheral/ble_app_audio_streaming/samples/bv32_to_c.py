import sys,os

if len(sys.argv) < 2:
    print "Usage: " + sys.argv[0] + "file.bv32"
    exit(1)
    
fin = open(sys.argv[1], "rb")

contents = fin.read()
output   = ""
hex_str  = ""
count    = 0

fin.close()



for s in contents:
    hex_str += str(hex(ord(s))) + ", "
    count   += 1

output += "#include <stdint.h>\r\n\r\n"
output += "const uint8_t sample_" + sys.argv[1].replace(".bv32", "") + "[" + str(count) + "] = {"
output += hex_str
output += "};\r\n"

fout = open(sys.argv[1].replace(".bv32", ".c"), "w")
fout.write(output)
fout.close()

