#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include) && __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include </home/daniel/reteneurale/GVFlib/filesystem.hpp>
namespace fs = ghc::filesystem;
#endif

#include <string.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <sys/stat.h>
#include <bits/stdc++.h>
#include <cmath>
#include <ctime>
//#include "mat.h"  // libraria para salvar arquivo em formato .mat
/*
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
*/
#define BUFSIZE 256
using namespace std;
using std::string;
//using namespace cv;
//MATFile *pmatX, *pmaty;
//mxArray *X; // matrix di "sizeDir" input (imagens desenroladas em 1x4096);

ofstream yFile("ydata.txt"); // file com dados de output (labels)
ofstream XFile("Xdata.txt"), outFileXY("merged.txt"), XFileT("test.txt"); // file com dados para treino
ifstream inFileX, inFileY;
//ofstream Xteste("Xteste2.txt"); // file dados para teste
char str[BUFSIZE];
int sizeDir, state, status, numInput = 0, numeroTeste = 0, k = 0, files = 0; // se sorriso state = 1; sem sorriso state = 0

vector<int> yLabels; // vetor de output (elementos = 0, 1, 2)
vector<float> prec;
//std::vector <int> ex, test; // matrix di "sizeDir" input (imagens desenroladas em 1x4096);
std::vector <float> xfiles, yfiles;
//vector<vector<int>> X;
vector<vector<float>> X;

//////////////////////////// funçoes //////////////
bool ordine (vector<string>& files, string path, int frames);
void readingFiles (string & line, int & state);
vector<int> convert(string line, int n);
void concatenate(string x, string y, string dirFaces, int state);
void process(int sta, int files);
//////////////////////////////////////////////////
string nameOfFile(string s){
	string res = "";
	size_t posI = s.find("file"); // index da primeira occorrencia de "faces" a partir do inicio da string
	size_t posF = s.rfind("txt"); // index da primeira ocorrencia de "pgm" começando do fim
	//s.replace(posF,3,"jpg");
	res = s.substr((posI)); //posF+2
return res;
}

void concatenate(string x, string y, string dirFaces, int state){

outFileXY.clear();
inFileX.clear();
inFileY.clear();

if(x.compare("xfile0.txt")==0 && state == 0){ 
k = 1;
cout << "k = 1!! "<< endl;
}
string st = dirFaces + x;
cout << "String st per X : " << st << endl;
inFileX.open(st);
st = dirFaces + y;
cout << "String st per y : " << st << endl;
inFileY.open(st);

outFileXY.open("merged.txt");

if(k==1) {
outFileXY << 5 << endl;
outFileXY.clear();
outFileXY.close();
outFileXY.open("merged.txt");
string line;
/*
while(getline(inFileX,line)){
		//cout << "Line de inFileX : " << line << endl;
		outFileXY << line << endl; 
}
*/
while(getline(inFileY,line)){
		//cout << "Line de inFileY : " << line << endl;
		outFileXY << line << endl; 
}
}
//else outFileXY << inFileX.rdbuf() << inFileY.rdbuf();
else outFileXY << inFileY.rdbuf();

outFileXY.close();
inFileX.close();
inFileY.close();
if(state < 3) yFile << state << endl;

process(state, files);
files++;
}

void process(int sta, int files){ // FUNZIONE********************************
    
string line;
int size = 130;
int n = 0;
ifstream ifile;
ifile.open("merged.txt");
prec.clear();

if(ifile.is_open() && k == 1) cout << "file merged open!" << endl;
if ( ifile.peek() == std::ifstream::traits_type::eof() ) cout << "File merge is empty!!" << endl;

if(k==1){
XFile.open("Xdata.txt");
XFile << 5 << endl;
XFile.close();
XFile.clear();

XFileT.open("test.txt");
XFileT << 5 << endl;
XFileT.close();
XFileT.clear();
}
if(sta < 3)
XFile.open("Xdata.txt", std::ios_base::app);
else XFileT.open("test.txt", std::ios_base::app);
cout << "State : " << sta << endl;	
	while(getline(ifile,line)){
        prec.push_back(stof(line));
        
			if(sta < 3){
                if(n == size - 1)  XFile << line << ", " << (stof(line) - prec[n-1]) << endl; //// mettere la VELOCITA!!!
                else{
                    if(n>=1 && n != 130)
                    XFile << line << ", " << (stof(line) - prec[n-1]) << ", ";
                    else XFile << line << ", 0, ";
                    }
            }
			else{
                if(n == size - 1) XFileT << line << ", " << (stof(line) - prec[n-1]) << endl; //// mettere la VELOCITA!!! 
                else{
                    if(n>=1 && n != 130)
                    XFileT << line << ", " << (stof(line) - prec[n-1]) << ", ";
                    else XFileT << line << ", 0, ";
                }
			}
		//}
	n++;	
	}
	
if(sta < 3) XFile.close();
else XFileT.close();
ifile.close();
ifile.clear();
k = 0;
}

void readingFiles (int state) {
int s = 0;
string dirLabel = "";
string dirFaces = "./Generated/";
string line = "";

switch (state){
    case 0:
        dirFaces = dirFaces + "braccia1x/";
        break;
    case 1:
        dirFaces = dirFaces + "maneboca1x/";
        break;
    case 2:
        dirFaces = dirFaces + "puntare1x/";
        break;
        case 3:
            dirFaces = "./";
            dirFaces =  dirFaces + "Templates/braccia1x/";
            break;
        case 4:
            dirFaces = "./";
            dirFaces =  dirFaces + "Templates/maneboca1x/";
            break;
        case 5:
            dirFaces = "./";
            dirFaces =  dirFaces + "Templates/puntare1x/";
            break;
    case 6:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45d1xCla/";
break;
    case 7:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45d2xCla/";
break;
    case 8:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45s1xCla/";
break;
    case 9:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45s2xCla/";
break;


    case 10:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45d1xCla/";
break;
    case 11:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45d2xCla/";
break;
    case 12:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45s1xCla/";
break;
    case 13:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45s2xCla/";
break;
    case 14:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45d1xCla/";
break;
    case 15:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45d2xCla/";
break;
    case 16:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45s1xCla/";
break;
    case 17:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45s2xCla/";
break;
    case 18:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45d1xRic/";
break;
    case 19:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45d2xRic/";
break;
    case 20:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45s1xRic/";
break;
    case 21:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/braccia45s2xRic/";
break;
    case 22:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45d1xRic/";
break;
    case 23:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45d2xRic/";
break;
    case 24:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45s1xRic/";
break;
    case 25:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/maneboca45s2xRic/";
break;
    case 26:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45d1xRic/";
break;
    case 27:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45d2xRic/";
break;
    case 28:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45s1xRic/";
break;
    case 29:
dirFaces = "./";
dirFaces =  dirFaces + "Templates/puntare45s2xRic/";
break;

} // SWITCH

cout << "Sorgente : " << dirFaces << endl;
sizeDir = 0;
for(auto& p : ghc::filesystem::directory_iterator(dirFaces)){ // tamanho do nosso diretorio de imagens (quantidade de input)
sizeDir += 1;
}
cout << "Misura dell'archivio : " << sizeDir << endl;
//while (getline(inFile,line)){
//size_t sizeL = line.length();
//line.erase(sizeL-1, 1);
//cout << "Tamanho de nossa string LINE : " << line.length() << "\n";
s = 0;
int z = 0;
	while(z < sizeDir/2){
			string x = "xfile" + to_string(z) + ".txt";
			string y = "yfile" + to_string(z) + ".txt";
            if(state >= 6){
                x = "xfileTemp.txt";
                y = "yfileTemp.txt";
            }
			concatenate(x,y,dirFaces, state);
			z++;
	}
}

int main(){
srand(time(NULL));

for(int i = 0 ; i < 30 ; i++){
readingFiles(i);
inFileX.close();
inFileX.clear();
inFileY.close();
inFileY.clear();
outFileXY.close();
outFileXY.clear();

files = 0;
cout << "File " << i << " compilado!\n";
}

ifstream fileXXX("Xdata.txt");
ifstream fileT("test.txt");
int si = 0;
string line;
while(getline(fileXXX,line))
	si++;
	cout << "misura del Xdata : " << si << endl;
si = 0 ;
while(getline(fileT,line))
	si++;
	cout << "misura del dataTest : " << si << endl;

}

