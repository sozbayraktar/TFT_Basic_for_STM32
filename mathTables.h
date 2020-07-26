
const unsigned char cosineTable[46]=  //0..45 derece açilarin cosinus degreleri (x100 olarak)
{100,100,100,100,100,100,99,99,99,99,98,98,98,97,97,97,
 96 ,96 ,95 ,95 ,94 ,93 ,93,92,91,90,90,89,88,87,87,86,
 85,84,83,82,81,81,80,79,77,75,74,73,72,71
 };
const unsigned char* pCosineTable=&cosineTable[0];
 
const unsigned char sineTable[46]=  //0..45 derece açilarin sinus degreleri (x100 olarak)
{0 ,2 ,3 ,5 ,7 ,9 ,10,12,14,16,17,19,21,22,24,26,
 28,29,31,33,34,36,37,39,41,42,44,45,47,48,50,52,
 53,54,56,57,59,60,62,63,64,66,67,68,69,71
 };
const unsigned char* pSineTable=&sineTable[0];
