//AES.C

/*-----------------------------------------------

  名称：AES算法的C语言实现

  编译环境：Microsoft Visual Studio 2010

/*-----------------------------------------------*/

#include <stdio.h>

#include <stdlib.h>

unsigned char xtime (unsigned char input);

void keyexpansion(unsigned char S_BOX[][16], unsigned char keys[][44]);

void bytesub(unsigned char S_BOX[][16], unsigned char B[][4]);

void shiftrow(unsigned char B[][4]);

void mixcolumn(unsigned char input[][4]);

void invbytesub(unsigned char N_S_BOX[][16], unsigned char B[][4]);

void invshiftrow(unsigned char B[][4]);

void invmixcolum(unsigned char input[][4]);

int encrypt(unsigned char S_BOX[][16]);

int decrypt(unsigned char S_BOX[][16], unsigned char N_S_BOX[][16]);

 

//密钥生成

void keyexpansion(unsigned char S_BOX[][16], unsigned char keys[][44])

{
    unsigned char Rcon[11] = {0, 1, 2, 4, 8, 16, 32, 64, 128, 27, 54};

    unsigned char past[4];

    register int i, j;

    printf("Please enter the key:\n");

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            scanf("%x", &keys[j][i]);

    for(i = 4; i <= 43; i ++)             //make the other 40 keys

    {
        if(i % 4 == 0)                    //如果能被4整除，特殊处理
        {
            for(j = 1; j <= 4; j ++)      //把前一个密钥移位赋值给数组

                past[j - 1] = keys[j % 4][i -1];

            for(j = 0; j <= 3; j ++)
            {
                if(j == 0)
                    keys[j][i] = S_BOX[past[j] / 16][past[j] % 16] ^ Rcon[i / 4] ^ keys[j][i - 4];
                else
                    keys[j][i] = S_BOX[past[j] / 16][past[j] % 16] ^ keys[j][i - 4];
            }

        }
        else
        {
            for(j = 0; j <= 3; j ++)
            {
                keys[j][i] = keys[j][i - 4] ^ keys[j][i - 1];
            }

        }

    }

}

 

//列混淆运算用到的乘2函数
unsigned char xtime (unsigned char input)    // x乘法('02'乘法)
{
    int temp;

    temp = input << 1;

    if(input & 0x80)

    {

        temp ^= 0x1b;

    }

    return temp;
}

//列混淆运算
void mixcolumn(unsigned char input[][4])  //列混淆
{
    int i, j;

    unsigned char output[4][4];

    for(j = 0; j <= 3; j++)

        for(i = 0; i <= 3; i++)

            output[i][j] = xtime(input[i%4][j]) //0x02乘法

                           ^ ( input[ ( i + 1 ) % 4][j] ^ xtime( input[ ( i + 1 ) % 4][j] ) ) //0x03乘法

                           ^ input[ ( i + 2 ) % 4][j]  //0x01乘法

                           ^ input[ ( i + 3 ) % 4][j]; //0x01乘法

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            input[i][j] = output[i][j];

}

//行移位
void shiftrow(unsigned char B[][4])
{
    int i, temp;

    temp = B[1][0];

    for(i = 0; i <= 2; i ++)

        B[1][i] = B[1][i + 1];

    B[1][3] = temp;

    for(i = 0; i <= 1; i ++)
    {
        temp = B[2][i];
        B[2][i] = B[2][i + 2];
        B[2][i + 2] = temp;
    }

 

    temp = B[3][3];

    for(i = 3; i >= 1; i --)

        B[3][i] = B[3][i - 1];

    B[3][0] = temp;

}

//字节变换
void bytesub(unsigned char S_BOX[][16], unsigned char B[][4])
{
    register int i, j;

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            B[i][j] = S_BOX[B[i][j] / 16][B[i][j] % 16];
}

//逆行移位
void invshiftrow(unsigned char B[][4])
{
    int i, temp;

    temp = B[1][3];

    for(i = 3; i >= 1; i --)

        B[1][i] = B[1][i - 1];

    B[1][0] = temp;

    for(i = 0; i <= 1; i ++)
    {
        temp = B[2][i];

        B[2][i] = B[2][i + 2];

        B[2][i + 2] = temp;
    }

 

    temp = B[3][0];

    for(i = 0; i <= 2; i ++)

        B[3][i] = B[3][i + 1];

    B[3][3] = temp;

}

//逆列混淆运算

void invmixcolum(unsigned char input[][4])
{
    int i, j;

    unsigned char output[4][4];

    for(j = 0; j < 4; j++)

        for(i = 0; i < 4; i++)

            output[i][j] = (xtime(xtime(xtime(input[i % 4][j]))) ^ xtime(xtime(input[i % 4][j])) ^ xtime(input[i % 4][j])) //0x0E乘法

                           ^ (xtime(xtime(xtime(input[ ( i + 1 ) % 4][j]))) ^ xtime(input[ ( i + 1 ) % 4][j]) ^ input[ ( i + 1 ) % 4][j]) //0x0B乘法

                           ^ (xtime(xtime(xtime(input[ ( i + 2 ) % 4][j]))) ^ xtime(xtime(input[ ( i + 2 ) % 4][j])) ^ input[ ( i + 2 ) % 4][j]) //0x0D乘法

                           ^ (xtime(xtime(xtime(input[ ( i + 3 ) % 4][j]))) ^ input[ ( i + 3 ) % 4][j]); //0x09乘法

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            input[i][j] = output[i][j];

}

//逆字节变换
void invbytesub(unsigned char N_S_BOX[][16], unsigned char B[][4])
{
    register int i, j;

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            B[i][j] = N_S_BOX[B[i][j] / 16][B[i][j] % 16];

}

 
int encrypt(unsigned char S_BOX[][16])
{
    // e是明文，B是密文
    unsigned char e, B[4][4];
    unsigned char keys[4][44];

    int i, j;
    int level;

    printf("Please enter the plaintext:\n");

    for(i = 0; i <= 3; i ++)
    {
        for(j = 0; j <= 3; j ++)
        {
            scanf("%x", &e);
            B[j][i] = e;
        }
    }

    getchar();

    //生成密钥 keys
    keyexpansion(S_BOX, keys);

    for(i = 0; i <= 3; i ++)
    {
        for(j = 0; j <= 3; j ++)
        {
            B[i][j] ^= keys[i][j];
        }
    }


    for(level = 1; level <= 9; level ++)    //1到9轮循环

    {
        bytesub(S_BOX, B);

        shiftrow(B);

        mixcolumn(B);

        for(i = 0; i <= 3; i ++)

            for(j = 0; j <= 3; j ++)

                B[i][j] ^= keys[i][level * 4 + j];

    }

    bytesub(S_BOX, B);               //第10轮循环

    shiftrow(B);

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            B[i][j] ^= keys[i][40 + j];

 

    printf("The ciphertext is：\n");

    for(i = 0; i <= 3; i ++)
    {
        for(j = 0; j <= 3; j ++)
        {
            printf("%02x ", B[j][i]);
        }
    }

    putchar('\n');
    return 0;

}

int decrypt(unsigned char S_BOX[][16], unsigned char N_S_BOX[][16])
{
    // B是密文，也作为明文输出
    unsigned char B[4][4];
    unsigned char keys[4][44];
    int temp, i, j;
    int level;

    printf("Please enter the plaintext:\n");

    for(i = 0; i <= 3; i ++)
    {
        for(j = 0; j <= 3; j ++)
        {
            scanf("%x", &temp);
            B[j][i] = temp;
        }
    }

    keyexpansion(S_BOX, keys);

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            B[i][j] ^= keys[i][j + 40];

    for(level = 1; level <= 9; level ++)
    {
        invshiftrow(B);

        invbytesub(N_S_BOX, B);

        for(i = 0; i <= 3; i ++)

            for(j = 0; j <= 3; j ++)

                B[i][j] ^= keys[i][40 - level * 4 + j];

        invmixcolum(B);
    }


    invshiftrow(B);
    invbytesub(N_S_BOX, B);

    for(i = 0; i <= 3; i ++)

        for(j = 0; j <= 3; j ++)

            B[i][j] ^= keys[i][j];

    printf("The output information as:\n");

    for(i = 0; i <= 3; i ++)
    {
        for(j = 0; j <= 3; j ++)
            printf("%02x ", B[j][i]);
    }
    putchar('\n');

    return 0;

 

}

int main()
{
    unsigned char S_BOX[16][16] =
    {
        0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5,
        0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
        0xCA, 0x82, 0xC9, 0x7D, 0xFA, 0x59, 0x47, 0xF0,
        0xAD, 0xD4, 0xA2, 0xAF, 0x9C, 0xA4, 0x72, 0xC0,
        0xB7, 0xFD, 0x93, 0x26, 0x36, 0x3F, 0xF7, 0xCC,
        0x34, 0xA5, 0xE5, 0xF1, 0x71, 0xD8, 0x31, 0x15,
        0x04, 0xC7, 0x23, 0xC3, 0x18, 0x96, 0x05, 0x9A,
        0x07, 0x12, 0x80, 0xE2, 0xEB, 0x27, 0xB2, 0x75,
        0x09, 0x83, 0x2C, 0x1A, 0x1B, 0x6E, 0x5A, 0xA0,
        0x52, 0x3B, 0xD6, 0xB3, 0x29, 0xE3, 0x2F, 0x84,
        0x53, 0xD1, 0x00, 0xED, 0x20, 0xFC, 0xB1, 0x5B,
        0x6A, 0xCB, 0xBE, 0x39, 0x4A, 0x4C, 0x58, 0xCF,
        0xD0, 0xEF, 0xAA, 0xFB, 0x43, 0x4D, 0x33, 0x85,
        0x45, 0xF9, 0x02, 0x7F, 0x50, 0x3C, 0x9F, 0xA8,
        0x51, 0xA3, 0x40, 0x8F, 0x92, 0x9D, 0x38, 0xF5,
        0xBC, 0xB6, 0xDA, 0x21, 0x10, 0xFF, 0xF3, 0xD2,
        0xCD, 0x0C, 0x13, 0xEC, 0x5F, 0x97, 0x44, 0x17,
        0xC4, 0xA7, 0x7E, 0x3D, 0x64, 0x5D, 0x19, 0x73,
        0x60, 0x81, 0x4F, 0xDC, 0x22, 0x2A, 0x90, 0x88,
        0x46, 0xEE, 0xB8, 0x14, 0xDE, 0x5E, 0x0B, 0xDB,
        0xC2, 0xD3, 0xAC, 0x62, 0x91, 0x95, 0xE4, 0x79,
        0xE7, 0xC8, 0x37, 0x6D, 0x8D, 0xD5, 0x4E, 0xA9,
        0x6C, 0x56, 0xF4, 0xEA, 0x65, 0x7A, 0xAE, 0x08,
        0xBA, 0x78, 0x25, 0x2E, 0x1C, 0xA6, 0xB4, 0xC6,
        0xE8, 0xDD, 0x74, 0x1F, 0x4B, 0xBD, 0x8B, 0x8A,
        0x70, 0x3E, 0xB5, 0x66, 0x48, 0x03, 0xF6, 0x0E,
        0x61, 0x35, 0x57, 0xB9, 0x86, 0xC1, 0x1D, 0x9E,
        0xE1, 0xF8, 0x98, 0x11, 0x69, 0xD9, 0x8E, 0x94,
        0x9B, 0x1E, 0x87, 0xE9, 0xCE, 0x55, 0x28, 0xDF,
        0x8C, 0xA1, 0x89, 0x0D, 0xBF, 0xE6, 0x42, 0x68,
        0x41, 0x99, 0x2D, 0x0F, 0xB0, 0x54, 0xBB, 0x16
    };

    unsigned char N_S_BOX[16][16] =
    {
        0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38,
        0xBF, 0x40, 0xA3, 0x9E, 0x81, 0xF3, 0xD7, 0xFB,
        0x7C, 0xE3, 0x39, 0x82, 0x9B, 0x2F, 0xFF, 0x87,
        0x34, 0x8E, 0x43, 0x44, 0xC4, 0xDE, 0xE9, 0xCB,
        0x54, 0x7B, 0x94, 0x32, 0xA6, 0xC2, 0x23, 0x3D,
        0xEE, 0x4C, 0x95, 0x0B, 0x42, 0xFA, 0xC3, 0x4E,
        0x08, 0x2E, 0xA1, 0x66, 0x28, 0xD9, 0x24, 0xB2,
        0x76, 0x5B, 0xA2, 0x49, 0x6D, 0x8B, 0xD1, 0x25,
        0x72, 0xF8, 0xF6, 0x64, 0x86, 0x68, 0x98, 0x16,
        0xD4, 0xA4, 0x5C, 0xCC, 0x5D, 0x65, 0xB6, 0x92,
        0x6C, 0x70, 0x48, 0x50, 0xFD, 0xED, 0xB9, 0xDA,
        0x5E, 0x15, 0x46, 0x57, 0xA7, 0x8D, 0x9D, 0x84,
        0x90, 0xD8, 0xAB, 0x00, 0x8C, 0xBC, 0xD3, 0x0A,
        0xF7, 0xE4, 0x58, 0x05, 0xB8, 0xB3, 0x45, 0x06,
        0xD0, 0x2C, 0x1E, 0x8F, 0xCA, 0x3F, 0x0F, 0x02,
        0xC1, 0xAF, 0xBD, 0x03, 0x01, 0x13, 0x8A, 0x6B,
        0x3A, 0x91, 0x11, 0x41, 0x4F, 0x67, 0xDC, 0xEA,
        0x97, 0xF2, 0xCF, 0xCE, 0xF0, 0xB4, 0xE6, 0x73,
        0x96, 0xAC, 0x74, 0x22, 0xE7, 0xAD, 0x35, 0x85,
        0xE2, 0xF9, 0x37, 0xE8, 0x1C, 0x75, 0xDF, 0x6E,
        0x47, 0xF1, 0x1A, 0x71, 0x1D, 0x29, 0xC5, 0x89,
        0x6F, 0xB7, 0x62, 0x0E, 0xAA, 0x18, 0xBE, 0x1B,
        0xFC, 0x56, 0x3E, 0x4B, 0xC6, 0xD2, 0x79, 0x20,
        0x9A, 0xDB, 0xC0, 0xFE, 0x78, 0xCD, 0x5A, 0xF4,
        0x1F, 0xDD, 0xA8, 0x33, 0x88, 0x07, 0xC7, 0x31,
        0xB1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xEC, 0x5F,
        0x60, 0x51, 0x7F, 0xA9, 0x19, 0xB5, 0x4A, 0x0D,
        0x2D, 0xE5, 0x7A, 0x9F, 0x93, 0xC9, 0x9C, 0xEF,
        0xA0, 0xE0, 0x3B, 0x4D, 0xAE, 0x2A, 0xF5, 0xB0,
        0xC8, 0xEB, 0xBB, 0x3C, 0x83, 0x53, 0x99, 0x61,
        0x17, 0x2B, 0x04, 0x7E, 0xBA, 0x77, 0xD6, 0x26,
        0xE1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0C, 0x7D
    };

    int choose;

    printf("This is C implementation of AES:\n");//打印标题

loop:

    printf("\nEncryption please enter: 1\n");
    printf("Decryption please input: 2\n");
    printf("Exit please input: 0 \n\n");
    printf("Please enter your choice:");
    scanf("%d", &choose);

    switch(choose)
    {
        case 1:
            //加密
            encrypt(S_BOX);
            goto loop;

        case 2:
            //解密
            decrypt(S_BOX, N_S_BOX);
            goto loop;

        case 0:
            exit(0);

        default :
            exit(0);
    }
    return 0;

}

//Test number

//Key

//2b 7e 15 16 28 ae d2 a6 ab f7 15 88 09 cf 4f 3c

//B

//32 43 f6 a8 88 5a 30 8d 31 31 98 a2 e0 37 07 34

//C

//39 25 84 1d 02 dc 09 fb dc 11 85 97 19 6a 0b 32