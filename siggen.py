import os
import struct
#void softSig(unsigned int startAddr, unsigned int length, FLASH_SIG_Type *pSig)
#{
    #FLASH_SIG_Type flashWord;
    #FLASH_SIG_Type refSignature = {0, 0, 0, 0};
    #FLASH_SIG_Type nextSign;
    #unsigned int* PageAddr = (unsigned int*)((unsigned int)startAddr);
 
    #for (unsigned int i = 0; i < (length >> 4); i++) {
        #flashWord.word0 = *PageAddr;
        #PageAddr++;
        #flashWord.word1 = *PageAddr;
        #PageAddr++;
        #flashWord.word2 = *PageAddr;
        #PageAddr++;
        #flashWord.word3 = *PageAddr;
        #PageAddr++;
 
        #//Update 128 bit signature
        #nextSign.word0 = flashWord.word0 ^ refSignature.word0 >> 1 ^ refSignature.word1 << 31;
        #nextSign.word1 = flashWord.word1 ^ refSignature.word1 >> 1 ^ refSignature.word2 << 31;
        #nextSign.word2 = flashWord.word2 ^ refSignature.word2 >> 1 ^ refSignature.word3 << 31;
        #nextSign.word3 = flashWord.word3 ^ refSignature.word3 >> 1 ^
                         #(refSignature.word0 & 1 << 29) << 2 ^
                         #(refSignature.word0 & 1 << 27) << 4 ^
                         #(refSignature.word0 & 1 << 2) << 29 ^
                         #(refSignature.word0 & 1 << 0) << 31;
 
        #//Point to the calculated value
        #refSignature.word0 = nextSign.word0;
        #refSignature.word1 = nextSign.word1;
        #refSignature.word2 = nextSign.word2;
        #refSignature.word3 = nextSign.word3;
    #}
 
    #//Copy the reference signature to the result pointer
    #pSig->word0 = refSignature.word0;
    #pSig->word1 = refSignature.word1;
    #pSig->word2 = refSignature.word2;
    #pSig->word3 = refSignature.word3;
#}

filename = "SKYFalcon_Radio.bin"
f = open(filename, "r+");
size = os.path.getsize(filename)


class Sig:
    
    def __init__(self, w0 = 0, w1 = 0, w2 = 0, w3 = 0):
        self.word0 = w0
        self.word1 = w1
        self.word2 = w2
        self.word3 = w3
        
    def __repr__(self):
        return "%08x %08x %08x %08x" % (self.word0, self.word1, self.word2, self.word3)
    
refSignature = Sig()
nextSign = Sig()
flashWord = Sig()

def getword():
    d = f.read(4)
    return struct.unpack_from("I", d)[0]

size -= 0xE0
f.seek(0xE0)
i = 0

while i < (size>>4):
    flashWord.word0 = getword()
    flashWord.word1 = getword()
    flashWord.word2 = getword()
    flashWord.word3 = getword()

    nextSign.word0 = flashWord.word0 ^ (refSignature.word0 >> 1) ^ ((refSignature.word1 << 31)& 0xFFFFFFFF);
    nextSign.word1 = flashWord.word1 ^ (refSignature.word1 >> 1) ^ (refSignature.word2 << 31)& 0xFFFFFFFF;
    nextSign.word2 = flashWord.word2 ^ (refSignature.word2 >> 1) ^ (refSignature.word3 << 31)& 0xFFFFFFFF;
    nextSign.word3 = flashWord.word3 ^ (refSignature.word3 >> 1) ^ (((refSignature.word0 & (1 << 29)) << 2)& 0xFFFFFFFF) ^ \
                            (((refSignature.word0 & (1 << 27)) << 4)& 0xFFFFFFFF) ^ (((refSignature.word0 & (1 << 2)) << 29)& 0xFFFFFFFF) ^ (((refSignature.word0 & (1 << 0)) << 31)& 0xFFFFFFFF)
    refSignature.word0 = nextSign.word0;
    refSignature.word1 = nextSign.word1;
    refSignature.word2 = nextSign.word2;
    refSignature.word3 = nextSign.word3;
    
    i+= 1
    
    
print refSignature
f.seek(0xC0)
f.write(struct.pack("I", refSignature.word0))
f.write(struct.pack("I", refSignature.word1))
f.write(struct.pack("I", refSignature.word2))
f.write(struct.pack("I", refSignature.word3))
f.write(struct.pack("I", size))
    
