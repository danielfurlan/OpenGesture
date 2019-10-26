/**
 * Gesture Variation Follower class allows for early gesture recognition and variation tracking
 *
 * @details Original algorithm designed and implemented in 2011 at Ircam Centre Pompidou
 * by Baptiste Caramiaux and Nicola Montecchio. The library has been created and is maintained by Baptiste Caramiaux
 *
 * Copyright (C) 2015 Baptiste Caramiaux, Nicola Montecchio
 * STMS lab Ircam-CRNS-UPMC, University of Padova, Goldsmiths College University of London
 *
 * The library is under the GNU Lesser General Public License (LGPL v3)
 */
#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include) && __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include </home/daniel/reteneurale/GVFlib/filesystem.hpp>
namespace fs = ghc::filesystem;
#endif
#define PI 3.1415
// Result for BODY_25 (25 body parts consisting of COCO + foot)
// const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
//     {0,  "Nose"},
//     {1,  "Neck"},
//     {2,  "RShoulder"},
//     {3,  "RElbow"},
//     {4,  "RWrist"},
//     {5,  "LShoulder"},
//     {6,  "LElbow"},
//     {7,  "LWrist"},
//     {8,  "MidHip"},
//     {9,  "RHip"},
//     {10, "RKnee"},
//     {11, "RAnkle"},
//     {12, "LHip"},
//     {13, "LKnee"},
//     {14, "LAnkle"},
//     {15, "REye"},
//     {16, "LEye"},
//     {17, "REar"},
//     {18, "LEar"},
//     {19, "LBigToe"},
//     {20, "LSmallToe"},
//     {21, "LHeel"},
//     {22, "RBigToe"},
//     {23, "RSmallToe"},
//     {24, "RHeel"},
//     {25, "Background"}
// };


#include "GVF.h"
#include <string.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <algorithm>
#include <numeric>
#include <vector>
#include <nlohmann/json.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <bits/stdc++.h>
#include "matplotlibcpp.h"
#include <math.h>
//#include <map>
#include <cmath>
#include <Python.h>
//#include <boost/python.hpp>
#include <map>
//#include "lines3d_demo.py"
//#include </usr/lib/python2.7/dist-packages/mpl_toolkits/mplot3d>


using namespace std;
namespace plt = matplotlibcpp;
namespace plt2 = matplotlibcpp;

using json = nlohmann::json;
using std::string;


int a, frames = 0, totalFr = 10, numPunti, sizeDir = 0, Temp, elem, xGraph, dist = 0, progP, progN, block, Left, Right, Total = 0, qtdData;
char c;	
float Val, prevX, prevY, shoulder, arm, foreArm, angleRot, angleY, distSR = 0.0, angShoulderArm, angArmForeArm, primoAngolo,s = 0.0, lastXG, lastYG, lastX, lastY, lastXGS, lastYGS;
vector<float> v1;
vector<int> punti;
std::vector<string> ordineGesti;
int chosen[10]; // vettore che ricevrà quanti template sono stati "scelti" per i punti (dimensione arbitraria di 10 templates...)
string line, gesto, mode;
GVF **gvf = new GVF*[18];
GVF *gvf2 = new GVF();
//Gnuplot plot;
//std::vector<std::vector<float>> Xt, Yt, frt, Xg, Yg, frg;
std::vector<std::vector<float>> X, Y, fr;//, Xgen, Ygen;

std::vector<float> xPtsT, yPtsT, FR, x2d, y2d, z2d, xTb, yTb, zTb, xTm, yTm, zTm, xTp, yTp, zTp, punto, xgenGr, ygenGr;//, Xgen, Ygen;
//vector<float*> Xgen, Ygen;
//ofstream **Xgen = new ofstream*[1];
//ofstream **Ygen = new ofstream*[1];
ofstream XfileT, YfileT;
vector<ofstream> filX, filY;
ofstream ofsX, ofsY, ofsXY;
map<int,vector<float>> Xgen, Ygen, XgenS, YgenS;


GVF::GVFState state;
GVFOutcomes outcomes[18];
GVFGesture theGesture;
vector<GVFGesture> gesture;
void generate(float x, float y, string gesto);
void grafico(string mode);
void stato();
void registrare();
void follower();
void risultati();
void registrare2();
float resPa();
float resPe();
float kapa();
void Graph3D(float x, float y, float z); //, std::vector<std::vector<float>>, std::vector<std::vector<float>>, std::vector<std::vector<float>>);
float ang(float d, float limb);
float posizioneZ(float x, float y, float xRef, float yRef, float limb);
float dimension(float x1, float y1, float x2, float y2);
vector<float> rot(vector<float> P, float ang);
inline bool exists (const std::string& name);
void Files();
ofstream ifFiles(int y);
void openFiles(string Gesto);
void datasetSint();

float posizioneZ(float x1, float y1, float x2, float y2, float limb){
float z = 0.0;
float d;
	d = dimension(x1, y1, x2, y2);
	angleY = ang(x2-x1,d);
	if(angleY > 1.57) angleY = angleY - 2*(angleY - 1.57);
	angleRot = ang(abs(x2-x1),limb*cos(angleY));
	cout << "Angolo di ROT per membro " << limb << " : " << angleRot*180/PI << endl;
	cout << "Angolo di Y per membro " << limb << " : " << angleY*180/PI << endl;
	//if(d/limb < 0.97 || d/limb > 1.03) 
	z = limb*sin(angleRot)*cos(angleY); //angleRot not neglegible
	cout << "Valore per z : " << z << endl;
return z;
}

float dimension(float x1, float y1, float x2, float y2){
float d = 0.0;
	d = sqrt(pow((x1 - x2),2.0)+pow((y1 - y2),2.0));
return d;
}

float ang(float d, float limb){
float ang = 0.0;
	if(d<limb) ang = acos(d/limb);
	else ang = acos(limb/d);
return ang;
}

vector<float> rot(vector<float> P, float ang){
vector<float> P_rot;
float val = 0.0;
float mat[3][3] = {{cos(ang), 0, sin(ang)},{0, 1, 0},{-sin(ang), 0, cos(ang)}};
	for(int i = 0 ; i<3 ;  i++){
	val = 0.0;
		for(int j = 0 ; j<3 ; j++){
		val = val + mat[i][j]*P[j];
		}
	P_rot.push_back(val);
	} 
return P_rot;
}
//-----------------------------------------------------------------------------
void generate(float x, float y, string Gesto){
int r, f = 0;

string st;
xgenGr.clear();
ygenGr.clear();

lastX = x;
lastY = y;

if(elem == 0){
openFiles(Gesto);
}
//cout << "misura del Xgen : " << Xgen.size() << endl;
cout << "Valore x  : " << x << ". Gesto : " << Gesto << endl;
f=0;
while(f<2*qtdData){
//xgenGr.clear();
//ygenGr.clear();
if(f == qtdData) {
x = -x;
lastX = -lastX;
cout << "cambio di segnale! " << endl;
}
r = rand() % 6;
if(r < 3) r = -r;
	if(f%10 != 0){
	x = x + r;
	y = y + r;
	}
	else{
	x = lastX + r;
	y = lastY + r;
	}
	st = "./Generated/"+gesto+"/xfile" + to_string(f) + ".txt";
	ofsX.open(st, std::ios_base::app);
	cout << "file opened!!" << endl;
	ofsX << x << endl;
	st = "./Generated/"+gesto+"/yfile" + to_string(f) + ".txt";
	ofsY.open(st, std::ios_base::app);
	ofsY << y << endl;
	
	ofsX.close();
	ofsY.close();
	ofsXY.close();
//if(Gesto.find("Switched") != std::string::npos){
if(f>=qtdData){
cout << "Stiamo negli archivi inversi! x = " << x << endl; 
XgenS[f].push_back(x);
YgenS[f].push_back(y);//1000-y);
lastXGS = x;
lastYGS = y;
}
else{
Xgen[f].push_back(x);
Ygen[f].push_back(y);//1000-y);
lastXG = x;
lastYG = y;
}	
f++;
}
/*
if(elem == 127){
cout << "ultimo valore del vetore (19) per Xgen e XgenS: " << Xgen[19].back() << endl;// <<", " << XgenS[19].back() << endl;
cout << "ultimo valore del vetore (20) per Xgen e XgenS: " << XgenS[20].back() << endl; //", " << XgenS[20].value() << endl;
//cout << "ultimo valore del vetore (39) per Xgen e XgenS: " << Xgen[39].value() <<", " << XgenS[39].value() << endl;
}
*/
}
//-----------------------------------------------------------------------------
void openFiles(string Gesto){
int r, f = 0;
string st;
ofsX.clear();
ofsY.clear();
	while(f<2*qtdData){
	st = "./Generated/"+Gesto+"/xfile" + to_string(f) + ".txt";
	ofsX.open(st);
	cout << "file opened First time!!" << endl;
	st = "./Generated/"+Gesto+"/yfile" + to_string(f) + ".txt";
	ofsY.open(st);
	
	ofsX.close();
	ofsY.close();
	//Xgen.push_back(xgenGr);
	//Ygen.push_back(ygenGr);
	Xgen.insert({f,xgenGr});
	Ygen.insert({f,ygenGr});
	XgenS.insert({f,xgenGr});
	YgenS.insert({f,ygenGr});
	f++;
	}
	cout << "misura dei vetori di grafici (Xgen e XgenS) : " << Xgen.size() << ", " << Ygen.size() << endl;
}
//-----------------------------------------------------------------------------
void Files(){
int f = 0;
	while(f<qtdData){
	string s = "./Generated/"+gesto+"/xfile" + to_string(f) +  ".txt";
	ofsX.open(s);
	//Xgen[f] = new ofstream;
		//Xgen[f] = &ofsX;
		//push_back(ofsX);
	s = "./Generated/"+gesto+"/yfile" + to_string(f)  +  ".txt";
	ofsY.open(s);
	//Ygen[f] = new ofstream;
		//Ygen[f] = &ofsY;
		f++;
	ofsX.close();
	ofsY.close();
	}
}

//riccardo/puntareG/puntare45d1x
//---------------------------------PARSING JSON, ROTATING POINTS, FEEDING VECTORS-------------------------------------------------
void parseJsonVector (nlohmann::json j, vector<float>& vector, int numPunti, std::vector<int>& punti, int State, string gesto){
vector.clear(); // pulire il vettore
xPtsT.clear();
yPtsT.clear();
FR.clear();
punto.clear();

cout << "Questo è il valore di elem : " << elem << "\n";
//cout << "Questo è il nostro punto : " << punti[0] << "\n";
auto INI = j["/people/0/pose_keypoints_2d"_json_pointer];
// 		ITERANDO SU INI!!! 
//cout << "Primo elemento del file : " << INI[0] << "\n";
	for(int pt = 0 ; pt < numPunti ; pt++){	
		vector.clear();		// INI[3] = "x" del punto di riferimento (1, neck)
		
		float xRef = INI[3]; cout << "xRef : " << xRef << "\n";
		float yRef = INI[4];
		float x1 = INI[0];  // nose, to assess the sign of rotation (if positive or negative)
		float y1 = INI[1];
		float x2 = INI[6]; 
		float y2 = INI[7]; 
		float x3 = INI[9];
		float y3 = INI[10];
		float x4 = INI[12]; cout << "x4 crudo: " << x4 << "\n";
		float y4 = INI[13]; //cout << "y4 crudo: " << y4 << "\n";
		float x5 = INI[15]; //LEFT SHOULDER
		float y5 = INI[16]; //LEFT SHOULDER
		if(dist == 0) {
		// Per calcolare le misure delle spalle e braccia. Solo per il primo frame, che deve essere perpendicolare alla camera!!
			shoulder = dimension(xRef, yRef, x2, y2);
			arm = dimension(x2,y2,x3,y3);
			foreArm = dimension(x3,y3,x4,y4);
			angShoulderArm = atan(abs(x3-x2)/abs(y3-y2));
			cout << "angShoulderArm : " << angShoulderArm*180/PI << endl;
			angArmForeArm = atan(abs(x4-x3)/abs(y4-y3));
				if(x4>x3) angArmForeArm = -angArmForeArm;
			cout << "angArmForeArm : " << angArmForeArm*180/PI << endl;
		dist = 1;			
		}
		if(State == 2 && s == 0.0){
		s = dimension(x2,y2,xRef,yRef);
		primoAngolo = ang(s,shoulder);
		}
		x1 = x1 - xRef;
		x2 = x2 - xRef;
		y2 = y2 - yRef;
		x3 = x3 - xRef;
		y3 = y3 - yRef;
		x4 = x4 - xRef;
		y4 = y4 - yRef;
		x5 = x5 - xRef;
		y5 = y5 - yRef;
		//-------------------------------- OFSTREAM & SYTHETIC DATA --------------------------------
		string left = gesto + "/Switched";
		if(elem < 129){
		XfileT << x4 << endl;
		YfileT << y4 << endl;
		generate(x4,y4,gesto);
		//generate(-x4,y4,gesto);
		//generate(-x4,y4,left);		
		Total = elem;
		cout << "Total : " << Total << endl;
		lastX = x4;
		lastY = y4;
		}
		else{
			if(elem == 129){
			int f = 0;
			XfileT << x4 << endl;
			YfileT << y4 << endl;
			generate(x4,y4,gesto);
			cout << "Valore x4 prima di inviare al Generate Inverso : " << x4 << endl;
			//generate(-x4,y4,gesto);
			//generate(-x4,y4,left);
			lastX = x4;
			lastY = y4;
		Total = elem;
		cout << "Total2 : " << Total << endl;
			}
		}
		// -----------------------------------------------------------------------------------------
		cout << "x4 : " << x4 << endl;
		
		
		if(x1 > 0) Left = 1;
		else Right = 1;
		/*
		int pos = punti[pt]*3;
		float x = INI[pos];
		x = (x - xRef);
		float y = INI[pos+1];
		y = (y - yRef);
		*/
		float x = x4;
		float y = y4;
		cout << "x calibrato : " << x4 <<"\n";	
			
		if(x!=0) prevX = x;
		if(y!=0) prevY = y;
		float z2 = 0.0, z3 = 0.0, z4 = 0.0;
			//if(State == 2){ FORSE STIPULARE CHE UN UNICO ANGOLO DI ROT DAL PRIMO FRAME E NON CALCOLARE SEMPRE AD OGNI FRAME... VEDERE
			// ANCHE SE NON MODIFICA IL VALORE CALCOLARE QUESTA ROT DEI VALORI FACENDO UN CALCOLO AGGIUNTIVO DI OGNI PUNTO (2 + 3 PER AVERE 4)
				float prevDistSR = distSR;
				//cout << "prevDistSR : " << prevDistSR << endl;
				distSR = dimension(x2,y2,0,0);	//dist vista shoulder right
				float distA = dimension(x3,y3,x2,y2);	//dist vista arm
				float distF = dimension(x4,y4,x3,y3);	//dist vista foreArm
				float angS = ang(distSR, shoulder);
				float angA = ang(distA, arm);
				float angF = ang(distF, foreArm);
				
				//store.push_back(distSR);
				
				cout << "Spalle misurate : " << distSR <<  endl;
				cout << "Shoulder	 : " << shoulder<<endl;
				
				float distSL = dimension(x5,y5,0,0);	// dist vista shoulder LEFT 
				float angSL = ang(distSL, shoulder);
				
				if(distSR<shoulder) {
				angleRot = acos(distSR/shoulder);
				cout << "divisione dist/s: " << distSR/shoulder << endl;
				}
				else {
				angleRot = acos(shoulder/distSR);
				cout << "divisione s/dist: " << shoulder/distSR << endl;
				}
				punto.push_back(x4);
				punto.push_back(y4);
				punto.push_back(0.0);
				
				if(elem >= 1){
				cout << "Right : " << Right << endl;
				cout << "Left : " << Left << endl; 
				if(distSR/shoulder < 0.90 || distSR/shoulder > 1.10){	// if there is a not negletible rotation
					cout << "angolo : " << (angS*180.0 / PI) << endl;
					/*
					if(x2<0)
					x2 = -distSR/cos(angS);
					else x2 = distSR/cos(angS);
					
					cout << "Valore progN : " << progN << endl;
					cout << "Valore progP : " << progP << endl;
					*/
					cout << "Valore distSR - prevDistSR : " << distSR - prevDistSR << endl;
				if((block != 1 || Left == 1) && Right != 1){ 
						//if((distSR - prevDistSR < -0.03 || progN >= 3) || block == -1){	// variations of shoulders indicates a positive angle of right-arm movement. Thus, we shall substract x values. The subject may be rotated to the LEFT
						if(distSR - prevDistSR < -0.03 && progN <= 5){
						progN += 1;
							if(progP > 0)	progP -= 1;
						}
						if(distSR - prevDistSR > 0 && block != -1){
							if(progN > 0)	progN -= 1;
							if(progP < 5)	progP += 1;
						}
					if(progP == 5) block = 1;
					if(progN == 5) block = -1;
					
					z2 = posizioneZ(x2, y2, 0, 0, shoulder);					
					/*			 	
				 	if(elem < 40){
				 	if(angShoulderArm > 0) z3 = posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	else z3 = -posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	if(angArmForeArm < 0) z4 = z3 - posizioneZ(x4, y4, x3, y3, foreArm);
				 	else z4 = z3 + posizioneZ(x4, y4, x3, y3, foreArm);
				 	}
				 	*/
				 	//else{
				 	z3 = posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	z4 = z3 + posizioneZ(x4, y4, x3, y3, foreArm);
				 	//}
				 	
				 	punto[0] = x4;
				 	punto[1] = y4;
				 	punto[2] = z4;
				 	
				 	cout << "Prima rotazione : " << endl;
				 	cout << "valore xL : " << punto[0] << endl;
				 	cout << "valore yL : " << punto[1] << endl;
				 	cout << "valore zL : " << punto[2] << endl;
				 	
				 	if(angS > primoAngolo) {
				 	float dif = abs(angS - primoAngolo);
				 	angS = angS + 0.1*dif;
				 	//angS = 0.9*(primoAngolo + dif);
				 	cout << "angS = primoAngolo + dif : " << angS*180/PI << endl;
				 	}
				 	
				 	float difX = abs(x2 + cos(primoAngolo)*shoulder);
				 	cout << "diff : " << difX << endl;
				 	punto[0] = x4 - difX;
				 	cout << "valore xL, x4 - dif : " << punto[0] << endl;
				 	
				 	punto = rot(punto, -angS);
				 	cout << "Dopo rotazione : " << endl;
				 	cout << "valore xL : " << punto[0] << endl;			 	
				 	cout << "valore yL : " << punto[1] << endl;
				 	cout << "valore zL : " << punto[2] << endl;
				 	cout << "PrimoAngolo : " << primoAngolo*180/PI << endl;
					//}
					}
				if((block != -1 || Right == 1) && Left != 1){
					//if(distSR - prevDistSR > 0.03 || progP >= 3 || progN >= 3 || block == 1){	// variations indicate a negative angle of right-arm movement. The subject may be rotated to the RIGHT	
					if(progP == 5) block = 1;
					if(progN == 5) block = -1;
					if(distSR - prevDistSR > 0.03 && progP <= 5){
					progP += 1;
						if(progN > 0)	progN -= 1;
					}
					if(distSR - prevDistSR < 0 && block != 1){
						if(progP > 0)	progP -= 1;
						if(progN < 5)	progN += 1;
					}
					
					
				 	z2 = -posizioneZ(x2, y2, 0, 0, shoulder);
				 	/*
				 	if(elem < 25 || elem > 55){
				 	if(angShoulderArm > 0) z3 = -posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	else z3 = posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	if(angArmForeArm < 0) z4 = z3 + posizioneZ(x4, y4, x3, y3, foreArm);
				 	else z4 = z3 - posizioneZ(x4, y4, x3, y3, foreArm);
				 	}
				 	*/
				 	//else{
				 	z3 = posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	z4 = z3 + posizioneZ(x4, y4, x3, y3, foreArm);
				 	//}
				 	
				 	/*
				 	if(angShouderArm > 0) z3 = -posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	else z3 = posizioneZ(x3, y3, x2, y2, arm) + z2;
				 	if(angArmForeArm < 0) z4 = z3 + posizioneZ(x4, y4, x3, y3, foreArm);
				 	else z4 = z3 - posizioneZ(x4, y4, x3, y3, foreArm);
				 	*/
				 	punto[0] = x4;
				 	punto[1] = y4;
				 	punto[2] = z4;
				 	
				 	cout << "Prima rotazione : " << endl;
				 	cout << "valore xR : " << punto[0] << endl;
				 	cout << "valore yR : " << punto[1] << endl;
				 	cout << "valore zR : " << punto[2] << endl;
				 	
				 	if(angS < primoAngolo) {
				 	float dif = abs(angS - primoAngolo);
				 	angS = primoAngolo + 0.5*dif;
				 	//angS = 0.9*(primoAngolo + dif);
				 	//cout << "angS = primoAngolo + dif : " << angS*180/PI << endl;
				 	}
				 	float difX = abs(x2 + cos(primoAngolo)*shoulder);
				 	cout << "diff : " << difX << endl;
				 	punto[0] = x4 + difX;
				 	cout << "valore xR, x4 - dif : " << punto[0] << endl;
				 	punto = rot(punto, angS);
				 	cout << "Dopo rotazione : " << endl;
				 	cout << "valore xR : " << punto[0] << endl;
				 	cout << "valore yR : " << punto[1] << endl;
				 	cout << "valore zR : " << punto[2] << endl;
				 	cout << "PrimoAngolo : " << primoAngolo*180/PI << endl;
					//}
					}
					/*
					if(abs(x) <= 1.1*shoulder)	x = x - ((x)*cos(angleRot)) ;
					else 				x = x + ((x)*cos(angleRot)) ;
					cout << "x rotazionato : " << x << "\n";
					*/
				}
				}
				
			//}
			/*
		float z2 = posizioneZ(x2, y2, 0, 0, shoulder); cout << "valori z2 : " << z2 << "\n";
		float z3 = z2 + posizioneZ(x3, y3, x2, y2, arm); cout << "valori z3 : " << z3 << "\n";
		float z4 = z3 + posizioneZ(x4, y4, x3, y3, foreArm); cout << "valori z4 : " << z4 << "\n";
		*/		
		if(x==0){ 
		x = prevX;
		vector.push_back(punto[0]);
		}
		else{
		vector.push_back(punto[0]);
		}
		if(y==0) {
		y = prevY;
		vector.push_back(punto[1]);
		}
		else{
		vector.push_back(punto[1]);
		}			
		//vector.push_back(punto[2]);
		if(State == 1) {	
					
			gvf[pt]->addObservation(vector);
			if(xGraph == 0) Val = x;			
			xGraph = 1;
			//Graph3D(x,y,z4); //, Xt, Yt, frt);
			if(elem < 130){
			cout << "valore x ad aggiungere nel template : " << punto[0] << endl;
			x2d.push_back(punto[0]);
			y2d.push_back(1000-punto[1]);
			z2d.push_back(elem);
			if(gesto.compare("braccia1x")==0){
			xTb.push_back(punto[0]);
			yTb.push_back(1000-punto[1]);
			zTb.push_back(elem);
			}
			else{
				if(gesto.compare("maneboca1x")==0){
				xTm.push_back(punto[0]);
				yTm.push_back(1000-punto[1]);
				zTm.push_back(elem);
				}
				else{
				xTp.push_back(punto[0]);
				yTp.push_back(1000-punto[1]);
				zTp.push_back(elem);
				}
			}
			}
		}
		//cout << "x che sta essendo messo : " << x <<"\n";
		if(State == 2) {			
			gvf[pt] -> update(vector);			
			outcomes[pt] = gvf[pt]->update(vector);
			if(xGraph == 0) Val = x;			
			xGraph = 1;
			//Graph3D(x,y,z4); //, Xg, Yg, frg);
			x2d.push_back(punto[0]);
			y2d.push_back(1000-punto[1]);
			z2d.push_back(elem);
		}

	}
	cout << "shoulder : " << shoulder << "\n";
	cout << "arm : " << arm << "\n";
	cout << "foreArm : " << foreArm << "\n\n";
elem += 1;
//cout << " misura del vettore : " << vector.size() << "\n";
}

void Graph3D(float x, float y, float z) {//,  std::vector<std::vector<float>> xv, std::vector<std::vector<float>> yv, std::vector<std::vector<float>> zv){
	for(float we = Val -300.0 ; we <= Val + 300.0 ; we+=1){						
		if(x-we <= 1 && x-we > 0){
			cout << "valori della differenza è raggiunto < 1!\n"; 				
			xPtsT.push_back(x);	
			yPtsT.push_back(1000-y);	
			FR.push_back(z);		
		continue;
		}							
	xPtsT.push_back(we);	
	yPtsT.push_back(1000-y);	
	FR.push_back(0);
	}
X.push_back(xPtsT);
Y.push_back(yPtsT);
fr.push_back(FR);
}

string nameOfFile(string s){
	string res = "";
	size_t pos = s.find("0");
	res = s.substr(pos,12);
	//cout << "Questo è il RES prima di tutto : " << res << "\n";
	//cout << "res.length() " << res.length() << "\n";
	for(int itr = 0 ; itr < res.length() ; ++itr){
	//cout << "Questo è il carattere della posizione " << itr << " " << res.at(itr) << "\n";
		if(res.at(itr) != '0'){ 
		res.erase(0,itr);//res.erase(itr);
		return res; 
		}
	}
	res = '0';
	//size_t pos2 = res.find()
	//cout << "File name : " << res << "\n";
return res;
}

bool ordine (vector<string>& files, string path, int frames){
string numberName = nameOfFile(path);

//cout << "ordine function... Number name : " << numberName << "\n";
string fr = to_string(frames);
//cout << "number of frame : " << fr << "\n";
//cout << "misura sizeDir : " << sizeDir << "\n";
	//if(numberName.find(fr)!= std::string::npos){
	if(numberName.compare(fr) == 0){	
//	cout << "Found the name!\n\n";
	files.push_back(path);
	//cout << "File size : " << files.size() << "\n";
		//if(files.size()==sizeDir) frames = -1;
	return true;	
	}
return false;
}

inline bool exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}
//---------------------------------READING FILES-----------------------------------------

// 			iterando gli archivi dei frame 
void readingFiles (vector<float>& vector, int frames, int totalFrames, int State, string & gesto) {
// questa funzione è stata creata in modo a leggere gli archivi di una sola cartella ("output"). 

int s = 0;
int finalFrame = frames  + totalFrames;
//cout << "Valori del primo frame : " << frames << "\n";
//cout << "Total frames : " << totalFrames << "\n";
std::vector<std::string> files;
string dir = "/home/daniel/reteneurale/openpose/";
if(State == 1){
dir = dir + "output/coco/" + gesto;
}
else{
dir = dir + "gesto/coco/" + gesto;
}
//printf("Siamo Inserisceti nel readingFiles. Aspettando i dati...\n");
sizeDir = 0;
for(auto& p : ghc::filesystem::directory_iterator(dir)){ // Getting the size of our directory
sizeDir += 1;
}

	while(s == 0){
		for(auto& p : ghc::filesystem::directory_iterator(dir)){
		//cout << "nome dell'archivio : " << p.path() << "\n";
		if(ordine(files,p.path(),frames)){ //ordinando i files che non sono presi in ordini di frame... 
			if(frames == -1 || files.size() == sizeDir){
				s=1;
				break;
			}		
		frames++;
		}
		}
	}
/// trasferindo valori JSON al vetori
	for(int it = 0 ; it < files.size(); it++){
	ifstream t(files[it]);
		if(t){
			json j;
			t >> j;
			parseJsonVector(j, vector, numPunti, punti, State, gesto);
		}
	}
}
//---------------------------------REGISTRARE------------------------------------- 
void registrare(){
string st;
int gr = 0;
	xGraph = 0;
	progP = 1;
	progN = 1;
	for(int og = 0 ; og < numPunti ; og++)
	gvf[og]->setState(GVF::STATE_LEARNING);
	cout << "\n Stato attuale: " << gvf[0]->getState() << "\n";
	cout << "Inserisce il nome della scatola del gesto : \n";
	cin >> gesto;
	cout << "Inserisce R per comminciare a registrare\n";
	cin >> c;
if(c == 'r'){
	while(c=='r'){
	elem = 0;
	X.clear();
	Y.clear();
	fr.clear();
	x2d.clear();
	y2d.clear();
	z2d.clear();
	XfileT.clear();
	YfileT.clear();
	Xgen.clear();
	Ygen.clear();
	XgenS.clear();
	YgenS.clear();
	/*
	xT2d.clear();
	yT2d.clear();
	zT2d.clear();
	*/
	v1.clear(); // pulindo vetori per ricevere i nuovi dati del nuovo gesto.
	if(gr == 1){
		cout << "\n Stato attuale: " << gvf[0]->getState() << "\n";
		cout << "Inserisce il nome della scatola del gesto : \n";
		cin >> gesto;
		cout << "Inserisce R per comminciare a registrare\n";
		cin >> c;
	}
st = "./Generated/"+gesto+"/xfileTemp.txt";
XfileT.open(st);
st = "./Generated/"+gesto+"/yfileTemp.txt";
YfileT.open(st);
	for(int og = 0 ; og < numPunti ; og++)
	gvf[og]->startGesture();	
	ordineGesti.push_back(gesto);	
	readingFiles(v1, frames, totalFr, 1, gesto);
	cout << "\nCreando grafico...abbia pacienza\n"; 
/*
	plt::plot_surface(X, Y, fr);
	string sa = "./" + gesto + ".png";
	plt::save(sa);
	plt::show();
	plt::close();
*/
	int siz = Total;
	if(siz < 130){
	//if(siz < elem){
	int f = 0;
	/*
	st = "./Generated/"+gesto+"/xfileTemp.txt"; // + to_string(f) + ".txt";
	XfileT.open(st, std::ios_base::app);
	st = "./Generated/"+gesto+"/yfileTemp.txt"; // + to_string(f) + ".txt";
	YfileT.open(st, std::ios_base::app);
	*/
		//while (siz < 129){
		while (siz < 129){
		cout << "misura siz : " << siz << endl;
		cout << "misura elem : " << elem << endl;
		XfileT << lastX << endl; // lastX << endl;
		YfileT << lastY << endl; //lastY << endl;
		
		int last = z2d.back();
		z2d.push_back(last+1);
		f = 0;
			while(f<qtdData){
			// data sintetico
			
			st = "./Generated/"+gesto+"/xfile" + to_string(f) + ".txt";
			ofsX.open(st, std::ios_base::app);
			ofsX << Xgen[f].back() << endl; // lastXG << endl;
			st = "./Generated/"+gesto+"/yfile" + to_string(f) + ".txt";
			ofsY.open(st, std::ios_base::app);
			ofsY << Ygen[f].back() << endl; // lastYG << endl;
			
			ofsX.close();
			ofsY.close();
			//---------------------------- NON C'E PIU SWITCHED!!!!!!!!!!!!!-------------------------------
			
			st = "./Generated/"+gesto+"/xfile" + to_string(f) + ".txt";
			ofsX.open(st, std::ios_base::app);
			ofsX << XgenS[f].back() << endl; // lastXGS << endl;
			st = "./Generated/"+gesto+"/yfile" + to_string(f) + ".txt";
			ofsY.open(st, std::ios_base::app);
			ofsY << YgenS[f].back() << endl; //lastYGS << endl;
			
			ofsX.close();
			ofsY.close();
			
			Xgen[f].push_back(Xgen[f].back()); //lastXG); 		// X generated
			Ygen[f].push_back(Ygen[f].back());
			XgenS[f].push_back(XgenS[f].back()); //lastXGS);	// X generated	AND switched
			YgenS[f].push_back(YgenS[f].back()); //lastYGS);	// Y generated AND switched
			
			 //lastYG);		// Y generated 
			f++;
			cout << "misura Xgen[0] : " << Xgen[0].size() <<endl;
			}
		siz += 1;
		}
		//int last = z2d.back();
		//z2d.push_back(last+1);
		cout << "value of z2d : " << z2d.size() << endl;
		XfileT.close();
		YfileT.close();
		f = 0;
	}
	else {
		XfileT.close();
		YfileT.close();
		int f = 0;
	}

	//grafico(mode);
	cout << "gesto finito. Cosa vuoi fare?\n\n'r' per nuovo gesto\n'e' per uscire del modo registrare\n";
	cin >> c;
	gr = 1;	
	Total = 0;
	}
}	
	for(int w = 0 ; w <= Temp ;w++) // to initialize the chosen vector with ZEROS
	chosen[w] = 0;		
	stato();
}
void registrare2(){
	xGraph = 0;
	progP = 1;
	progN = 1;

	for(int og = 0 ; og < numPunti ; og++)
	gvf[og]->setState(GVF::STATE_LEARNING);
	cout << "\n Stato attuale: " << gvf[0]->getState() << "\n";
	cout << "Inserisce R per registrare i 3 gesti : \n";
	cin >> c;
	////////////////////////////////////// JUST FOR WORKING ON CODE /////////////////////////////
	if(c == 'r'){
	string diret = "/home/daniel/reteneurale/openpose/output/coco/diretto";
	for(auto& p : ghc::filesystem::directory_iterator(diret)){
	X.clear();
	Y.clear();
	fr.clear();
	x2d.clear();
	y2d.clear();
	z2d.clear();		
		gvf[0]->startGesture();
		string s = p.path();	
		s = s.substr(s.rfind("/")+1);
		cout << "new string : " << s << endl;
		ordineGesti.push_back(s);	
		readingFiles(v1, frames, totalFr, 1, s);
		elem = 0;
	}
	/*
		gvf[0]->startGesture();
		s = "maneboca1x";
		ordineGesti.push_back(s);	
		readingFiles(v1, frames, totalFr, 1, s);
		elem = 0;
	X.clear();
	Y.clear();
	fr.clear();
	x2d.clear();
	y2d.clear();
	z2d.clear();
		gvf[0]->startGesture();
		s = "puntare1x";
		ordineGesti.push_back(s);	
		readingFiles(v1, frames, totalFr, 1, s);
		
		Temp = gvf[0]->getNumberOfGestureTemplates();
		cout << "Numero di templates : " << Temp << "\n";
	//////////////////////////////////////////////////////////////////////////////////////////////	
	*/
	}
	Temp = gvf[0]->getNumberOfGestureTemplates();
		cout << "Numero di templates : " << Temp << "\n";
	for(int w = 0 ; w <= Temp ;w++) // to initialize the chosen vector with ZEROS
	chosen[w] = 0;		
	stato();
}
void datasetSint(){
string st;
int gr = 0;
	xGraph = 0;
	progP = 1;
	progN = 1;
	for(int og = 0 ; og < numPunti ; og++)
	gvf[og]->setState(GVF::STATE_LEARNING);
	cout << "\n Stato attuale: " << gvf[0]->getState() << "\n";
	cout << "Inserisce il numero di dataset che vuoi creare (superiore a 10) : \n" << endl;
	cin >> qtdData;
	cout << "Inserisce il nome della scatola del gesto : \n";
	cin >> gesto;

	cout << "Inserisce R per comminciare a registrare\n";
	cin >> c;
if(c == 'r'){
	while(c=='r'){
	elem = 0;
	X.clear();
	Y.clear();
	fr.clear();
	x2d.clear();
	y2d.clear();
	z2d.clear();
	XfileT.clear();
	YfileT.clear();
	Xgen.clear();
	Ygen.clear();
	XgenS.clear();
	YgenS.clear();
	v1.clear(); // pulindo vetori per ricevere i nuovi dati del nuovo gesto.
	if(gr == 1){
		cout << "\n Stato attuale: " << gvf[0]->getState() << "\n";
		cout << "Inserisce il nome della scatola del gesto : \n";
		cin >> gesto;
		cout << "Inserisce R per comminciare a registrare\n";
		cin >> c;
	}
st = "./Generated/"+gesto+"/xfileTemp.txt";
XfileT.open(st);
st = "./Generated/"+gesto+"/yfileTemp.txt";
YfileT.open(st);
	for(int og = 0 ; og < numPunti ; og++)
	gvf[og]->startGesture();	
	ordineGesti.push_back(gesto);	
	readingFiles(v1, frames, totalFr, 1, gesto);
	cout << "\nCreando grafico...abbia pacienza\n"; 
/*
	plt::plot_surface(X, Y, fr);
	string sa = "./" + gesto + ".png";
	plt::save(sa);
	plt::show();
	plt::close();
*/
	int siz = Total;
	if(siz < 130){
	//if(siz < elem){
	int f = 0;
	/*
	st = "./Generated/"+gesto+"/xfileTemp.txt"; // + to_string(f) + ".txt";
	XfileT.open(st, std::ios_base::app);
	st = "./Generated/"+gesto+"/yfileTemp.txt"; // + to_string(f) + ".txt";
	YfileT.open(st, std::ios_base::app);
	*/
		//while (siz < 129){
		while (siz < 129){
		cout << "misura siz : " << siz << endl;
		cout << "misura elem : " << elem << endl;
		XfileT << lastX << endl; // lastX << endl;
		YfileT << lastY << endl; //lastY << endl;
		cout << "XfileT e YfileT opperativi" << endl;
		int last = z2d.back();
		z2d.push_back(last+1);
		float lastx = x2d.back();
		x2d.push_back(lastx);
		y2d.push_back(y2d.back());
		f = 0;
			while(f<2*qtdData){
			// data sintetico
			if(f<qtdData){
			st = "./Generated/"+gesto+"/xfile" + to_string(f) + ".txt";
			ofsX.open(st, std::ios_base::app);
			ofsX << Xgen[f].back() << endl; // lastXG << endl;
			st = "./Generated/"+gesto+"/yfile" + to_string(f) + ".txt";
			ofsY.open(st, std::ios_base::app);
			ofsY << Ygen[f].back() << endl; // lastYG << endl;
			
			ofsX.close();
			ofsY.close();
			Xgen[f].push_back(Xgen[f].back()); //lastXG); 		// X generated
			Ygen[f].push_back(Ygen[f].back()); //lastYG);		// Y generated 
			}
			else{
			st = "./Generated/"+gesto+"/xfile" + to_string(f) + ".txt";
			ofsX.open(st, std::ios_base::app);
			ofsX << XgenS[f].back() << endl; // lastXGS << endl;
			st = "./Generated/"+gesto+"/yfile" + to_string(f) + ".txt";
			ofsY.open(st, std::ios_base::app);
			ofsY << YgenS[f].back() << endl; //lastYGS << endl;
			
			ofsX.close();
			ofsY.close();
			XgenS[f].push_back(XgenS[f].back()); //lastXGS);	// X generated	AND switched
			YgenS[f].push_back(YgenS[f].back()); //lastYGS);	// Y generated AND switched
			}
			
			f++;
			cout << "misura Xgen[0] : " << Xgen[0].size() <<endl;
			}
		siz += 1;
		}
		//int last = z2d.back();
		//z2d.push_back(last+1);
		cout << "value of z2d : " << z2d.size() << endl;
		XfileT.close();
		YfileT.close();
		f = 0;
	}
	else {
		XfileT.close();
		YfileT.close();
		int f = 0;
	}
	for (auto itr = x2d.begin();itr< x2d.end();itr++)
	cout << "valore zx2d : " << *itr << endl;
	cout << "valore size di x2d : " << x2d.size() << endl;
	//grafico(mode);
	cout << "gesto finito. Cosa vuoi fare?\n\n'r' per nuovo gesto\n'e' per uscire del modo registrare\n";
	cin >> c;
	gr = 1;	
	Total = 0;
	}
}	
	for(int w = 0 ; w <= Temp ;w++) // to initialize the chosen vector with ZEROS
	chosen[w] = 0;		
	stato();
}

void follower(){
	/*
	int temp = gvf[1]->getNumberOfGestureTemplates();
	for(int w = 0 ; w <= temp ;w++) 
	chosen[w] = 0;				// to initialize the chosen vector with ZEROS
	*/
	elem = 0;
	//store.clear();
	
	//dist = 0;
/*
	Xg.clear();
	Yg.clear();
	frg.clear();
*/

	for(int og = 0 ; og < numPunti ; og++)	
	gvf[og]->setState(GVF::STATE_FOLLOWING);

	cout << "Inserisce il nome della scatola del gesto : \n";
	cin >> gesto;
	cout << "\n Stato attuale: " << gvf[0]->getState() << "\n";
	/*cout << "\n values of Frames and totalFr: " << frames << " + " << totalFr ;
	cout << "Inserisce la posizione del primo frame\n";
	cin >> frames;
	cout << "Inserisce la quantita di frames\n";
	cin >> totalFr;
	*/	
	cout << "Inserisce R per comminciare a registrare the gesture variation\n";
	cin >> c;
int gr = 0;
	if(c == 'r'){
	while(c=='r'){
	elem = 0;
	X.clear();
	Y.clear();
	fr.clear();
	
	x2d.clear();
	y2d.clear();
	z2d.clear();
	//int temp = gvf[1]->getNumberOfGestureTemplates();
	for(int w = 0 ; w <= Temp ;w++) // to initialize the chosen vector with ZEROS
	chosen[w] = 0;
	v1.clear(); // pulindo vetori per ricevere i nuovi dati del nuovo gesto.
	if(gr == 1){
		cout << "Inserisce il nome della scatola del gesto : \n";
		cin >> gesto;
		/*		
		cout << "Inserisce la posizione del primo frame\n";
		cin >> frames;
		cout << "Inserisce la quantita di frames\n";
		cin >> totalFr;
		*/
		cout << "Inserisce R per comminciare a registrare\n";
		cin >> c;
	}	
	for(int og = 0 ; og < numPunti ; og++) 
	gvf[og]->startGesture();
	progP = 1;
	progN = 1;
	block = 0;
	Right = 0;
	Left = 0;
	s = 0.0;
	
	string dir = "/home/daniel/reteneurale/openpose/gesto/coco/" + gesto;
	
		while(!exists(dir)){	
			cout << "bad name for file..." << endl;
			cout << "Inserisce un nome correto di gesto ou 'e' per uscire. \n";
			cin >> gesto;
			if(gesto == "e") {
			stato();
			break;
			}
			dir = "/home/daniel/reteneurale/openpose/gesto/coco/" + gesto;
		}
	readingFiles(v1, frames, totalFr, 2, gesto);
	risultati();
	cout << "Creando grafico...abbia pacienza\n"; 
	//grafico(mode);
	cout << "Cosa vuoi fare?\n\n'r' per nuovo gesto\n'e' per uscire del modo registrare\n";
	cin >> c;
	gr = 1;	
	}
	for(int og = 0 ; og < numPunti ; og++) {
	//gvf[og]->startGesture();
	cout << "Templates per punto " << og << " : "<< "\n";
		//for(int w = 0 ; w < gvf[og]->getNumberOfGestureTemplates() ; w++)
		gesture = gvf[og]->getAllGestureTemplates();	
		cout <<	gesture.size() << "\n";
	}
	for(int og = 0 ; og < numPunti ; og++) 
	gvf[og]->setState(GVF::STATE_BYPASS);

	
	stato();
	}	
}
//----------------------------------------------------------------------
/*
void grafico(string mode){
string j;
int r;
r = rand() % qtdData + qtdData;
cout << "r prima tutto : " << r << endl;
if(qtdData - r < 10 && r >= 10) r = r - 10;
//else r = 0;
cout << "mode  = " << mode << endl;
plt::figure();
if(mode.compare("registrare") == 0){

	//-----------X---------------
	plt::subplot(1,3,1);
	//plt::xlabel("x");
	//plt::ylabel("time");
	plt::named_plot("Template",x2d,z2d); //xTb,zTb);
	//generated
		
	plt::plot(xTm,zTm);
	plt::plot(xTp,zTp);
	plt::plot(xTb,zTb);
	//-----------Y---------------

	plt::subplot(1,3,3);
	plt::named_plot("Gesture",z2d,y2d);
	//plt::xlabel("time");
	//plt::ylabel("y");
	//generated
	plt::plot(zTb,yTb);
	plt::plot(zTm,yTm);
	plt::plot(zTp,yTp);	
	//plt::plot(z2d,y2d);
	string ftwoD = "/home/daniel/reteneurale/GVFlib/" + gesto + ".png";
	plt::save(ftwoD);
	/*
	plt::plot_surface(X, fr, Y);
	string sa = "/home/airlab/openpose/gesto/coco/" +gesto + ".png";
	plt::save(sa);
////////////////////////////////QUA C'E LA FINE DI UN COMMENTARIO BLOCCO!
	//plt::show();	
	plt::close();
}
else{
cout << "mode = dataset" << endl << "r = " << r << endl;
	for(int w = r; w < r + 10 ; w++){
	
	//-----------X---------------
	if(w >= qtdData){
	plt::subplot(1,3,1);
	//plt::xlabel("x");
	//plt::ylabel("time");
	//plt::plot(x2d,z2d);
	plt::named_plot("Template",x2d,z2d); //xTb,zTb);
	//generated
	
	plt::plot(XgenS[w],z2d);	
	//-----------Y---------------
	
	//string xtwoD = "./" + gesto + "X2d.png";
	//plt::save(xtwoD);
	plt::subplot(1,3,3);
	plt::named_plot("Gesture",z2d,y2d);
	//plt::xlabel("time");
	//plt::ylabel("y");
	//generated
	plt::plot(z2d,YgenS[w]);	
	//plt::plot(z2d,y2d);
	
	//string ftwoD = "/home/airlab/ofxGVF/GVFlib/" + gesto + ".png";
	//plt::save(ftwoD);
	
	j = "/home/daniel/reteneurale/GVFlib/" + gesto + to_string(w) + ".png";
	plt::save(j);
	/*
	plt::plot_surface(X, fr, Y);
	string sa = "/home/airlab/openpose/gesto/coco/" +gesto + ".png";
	plt::save(sa);
//////////////////////////////////////////QUA C'E LA FINE DI UN COMMENTARIO BLOCCO!
	plt::close();
	}
	else{
	plt::subplot(1,3,1);
	//plt::xlabel("x");
	//plt::ylabel("time");
	//plt::plot(x2d,z2d);
	plt::named_plot("Template",x2d,z2d); //xTb,zTb);
	//generated
	
	plt::plot(Xgen[w],z2d);	
	//-----------Y---------------
	
	//string xtwoD = "./" + gesto + "X2d.png";
	//plt::save(xtwoD);
	plt::subplot(1,3,3);
	plt::named_plot("Gesture",z2d,y2d);
	//plt::xlabel("time");
	//plt::ylabel("y");
	//generated
	plt::plot(z2d,Ygen[w]);	
	//plt::plot(z2d,y2d);
	
	//string ftwoD = "/home/airlab/ofxGVF/GVFlib/" + gesto + ".png";
	//plt::save(ftwoD);
	
	j = "/home/daniel/reteneurale/GVFlib/" + gesto + to_string(w) + ".png";
	plt::save(j);
	/*
	plt::plot_surface(X, fr, Y);
	string sa = "/home/airlab/openpose/gesto/coco/" +gesto + ".png";
	plt::save(sa);
	//////////////////////////////// QUA C'E LA FINE DI UN COMMENTARIO BLOCCO!
	plt::close();
	}
	}
	//plt::show();	

	plt::close();
	}
}
//}
//
*/
void risultati (){
	for(int g = 0; g < numPunti; g++){
	cout << "\nGesto riconosciuto : " << outcomes[g].likeliestGesture << "\n";
	chosen[outcomes[g].likeliestGesture] += 1;
	cout << "Likelihoods : \n";	
	for(int j = 0 ; j <outcomes[g].likelihoods.size(); j++)	
	cout << outcomes[g].likelihoods[j] << "\n";
	cout << "Alighments : \n";	
	for(int j = 0 ; j <outcomes[g].alignments.size(); j++)	
	cout << outcomes[g].alignments[j] << "\n";	
	cout << "Dynamics : \n";	
	for(int j = 0 ; j <outcomes[g].dynamics.size(); j++){	
		for(int e = 0; e < outcomes[g].dynamics[j].size(); e++)		
		cout << outcomes[g].dynamics[j][e] << "\n";
	}
	}
	for(int w = 0; w <= Temp; w++) //gvf[1]->getNumberOfGestureTemplates(); w++)
	cout << "Il nostro vettore di likelest : " << chosen[w] << "\n";
	float k = kapa();
	cout << "\nValore Kapa : " << k << "\n";
	int idx, big = 0;
	int size = (int)(sizeof(chosen)/sizeof(chosen[0])); 
	for(int h = 0 ; h < size ; h++){
		if (chosen[h] > big){
		big = chosen[h];
		idx = h;	
		}
	}
	cout << "Gesto generale riconosciuto : " << idx << ", "<< ordineGesti[idx] <<"\n\n";	
}
//-------------------------------------------------------------------------
//Funzione per definire STATO
void stato (){
int i,e;
	
	cout << "Sceglie l'opzione: \n 1) Rigestrazione\n 2) Follower\n 3) Exit\n 4) Rimuovere gesto template\n 5) Vedere gesti\n 6) registrazione automatica\n 7) Creare dataset sintetico\n";
	cin >> i;
	cout << "\nHai scelto l'opzione " << i << "\n";


switch (i){

case 1 :
	cout << "\n...setting lo stato per registrare...\n ";
	mode = "registrare";
	registrare();
	break;
case 2 :
	cout << "\n...setting lo stato per follower...\n ";
	follower();
	break;	
case 3 :
	exit(0);
	break;
case 4 : {
	int gest = gvf[0]->getNumberOfGestureTemplates();
	cout << "Numero di templates (case4) : " << gest << " \n";
	cout << "Gesture template.size() : " << gesture.size() << "\n";
	cout << "Sceglie il gesto ad essere rimuovuto : \n";
	for(int w = 0; w <= Temp; w++)
	cout << w << ") " << ordineGesti[w] << "\n";
	cin >> e;
	for(int w = 0; w < numPunti; w++){
	gvf[w]->removeGestureTemplate(e);
	}
	auto itr = std::find(ordineGesti.begin(),ordineGesti.end(),ordineGesti[e]);
	if(itr != ordineGesti.end()) ordineGesti.erase(itr);
	//ordineGesti.erase(ordineGesti.begin(),ordineGesti.end(), e);
	stato();
	break;

}
case 5 : {
	int gest = gvf[0]->getNumberOfGestureTemplates();
	cout << "Numero templates : " << gest << "\n";
	cout << "Gesti templates : " << gesture.size() << "\n";
	cout << "\nGesti registrati : \n";
	for(int w = 0; w <= Temp; w++)
	cout << w << ") " << ordineGesti[w] << "\n";
	cout << "\n";
	stato();	
	break;
	}
case 6 : {
	cout << "\n...setting lo stato per registrare automatico...\n ";
	registrare2();
	break;
}
case 7 : {
	cout << "Dataset Sintetico.";
	mode = "data";
	datasetSint();
	break;
}
}
}
//---------------------------------------------------------------------
float kapa(){
float k = 0.0;
	float pa = resPa();	
	cout << "\nvalore Pa : " << pa;
	float pe = resPe();
	cout << "\nvalore Pe : " << pe << "\n\n";
if(pa == 1 || pe == 1) return k = 1;
k = (pa-pe)/(1-pe);
return k;
}

float resPa (){
float res = 0.0;
int temp = gvf[0]->getNumberOfGestureTemplates();
int sum = 0;	
	for (int a = 0; a < temp; a++){
	sum = sum + (chosen[a])*(chosen[a]);
	cout << "\nvalore chosen[a] : " << chosen[a] << "\n";	
	cout << "\nvalore punti : " << numPunti << "\n";	
	
	cout << "\nvalore sum : " << sum << "\n";	
	}
	sum = sum - numPunti;
	res = (float)sum/(numPunti*(numPunti-1));
return res;
}

float resPe(){
int temp = gvf[0]->getNumberOfGestureTemplates();
float pe = 0.0;
float val = 0.0;
	for(int a = 0; a < temp ; a++){	
	pe = pe + ((float)chosen[a]/numPunti)*((float)chosen[a]/numPunti);
	}
return pe;
}
//---------------------------MAIN--------------------------------------------------
int main(){
srand(time(NULL));
//plt.use('TkAgg');
//XfileT.open("./Generated/xfileTemp.txt");
//YfileT.open("./Generated/yfileTemp.txt");
int f = 0;
//Files();

	
int v;
//chosen.clear();
punti.clear();
cout << "Benvenuto\n";
cout << "Inserisce i punti che vuoi analisare e poi \"f\" per concludere\n";
cout << "{0,  \"Nose\"}\n{1,  \"Neck\"}\n{2,  \"RShoulder\"}\n{3,  \"RElbow\"}\n{4,  \"RWrist\"}\n{5,  \"LShoulder\"}\n{6,  \"LElbow\"}\n{7,  \"LWrist\"}\n{8,  \"MidHip\"}\n{9,  \"RHip\"}\n{11, \"RAnkle\"}\n{12, \"LHip\"}\n{13, \"LKnee\"}\n{14, \"LAnkle\"}\n{15, \"REye\"}\n{16, \"LEye\"}\n{17, \"REar\"}\n{18, \"LEar\"}\n";
	
	cin >> v;
	if(cin){	
		do{
		//int num = (int)v;
		punti.push_back(v);
		cin >> v;
		}	
		while(cin) ;
	}
	cin.clear();     
	cin.ignore(); 	
	numPunti = punti.size();
	cout << "dimension del vettore punti : " << numPunti << "\n";
	for (auto c = 0; c < punti.size(); c++)
	gvf[c] = new GVF;		
	
	stato();
}

GVF::GVF()
{
    config.inputDimensions   = 2;
    config.translate         = true;
    config.segmentation      = false;
    
    parameters.numberParticles       = 4000;
    parameters.tolerance             = 0.2f;
    parameters.resamplingThreshold   = 250;
    parameters.distribution          = 0.0f;
    parameters.alignmentVariance     = sqrt(0.1f);
    parameters.dynamicsVariance      = vector<float>(1,sqrt(0.001f));
    parameters.scalingsVariance      = vector<float>(1,sqrt(0.1f));
    parameters.rotationsVariance     = vector<float>(1,sqrt(0.0f));
    parameters.predictionSteps       = 1;
    parameters.dimWeights            = vector<float>(1,sqrt(1.0f));
    parameters.alignmentSpreadingCenter     = 0.0;
    parameters.alignmentSpreadingRange      = 0.2;
    parameters.dynamicsSpreadingCenter      = 1.0;
    parameters.dynamicsSpreadingRange       = 0.3;
    parameters.scalingsSpreadingCenter      = 1.0;
    parameters.scalingsSpreadingRange       = 0.3;
    parameters.rotationsSpreadingCenter     = 0.0;
    parameters.rotationsSpreadingRange      = 0.5;
    
    tolerancesetmanually = false;
    learningGesture = -1;
    
    normgen = std::mt19937(rd());
    rndnorm = new std::normal_distribution<float>(0.0,1.0);
    unifgen = std::default_random_engine(rd());
    rndunif = new std::uniform_real_distribution<float>(0.0,1.0);
    
}

////--------------------------------------------------------------
//GVF::GVF(GVFConfig _config){
//    setup(_config);
//}
//
////--------------------------------------------------------------
//GVF::GVF(GVFConfig _config, GVFParameters _parameters){
//    setup(_config, _parameters);
//}
//
////--------------------------------------------------------------
//void GVF::setup(){
//    
//    // use defualt parameters
//    GVFConfig defaultConfig;
//    
//    defaultConfig.inputDimensions   = 2;
//    defaultConfig.translate         = true;
//    defaultConfig.segmentation      = false;
//    
//    setup(defaultConfig);
//}
//
////--------------------------------------------------------------
//void GVF::setup(GVFConfig _config){
//    
//    clear(); // just in case
//    
//    learningGesture = -1;
//    
//    // Set configuration:
//    config      = _config;
//    
//    // default parameters
//    GVFParameters defaultParameters;
//    defaultParameters.numberParticles       = 1000;
//    defaultParameters.tolerance             = 0.2f;
//    defaultParameters.resamplingThreshold   = 250;
//    defaultParameters.distribution          = 0.0f;
//    defaultParameters.alignmentVariance     = sqrt(0.000001f);
//    defaultParameters.dynamicsVariance      = vector<float>(1,sqrt(0.001f));
//    defaultParameters.scalingsVariance      = vector<float>(1,sqrt(0.00001f));
//    defaultParameters.rotationsVariance     = vector<float>(1,sqrt(0.0f));
//    defaultParameters.predictionSteps       = 1;
//    defaultParameters.dimWeights            = vector<float>(1,sqrt(1.0f));
//    
//    // default spreading
//    defaultParameters.alignmentSpreadingCenter = 0.0;
//    defaultParameters.alignmentSpreadingRange  = 0.2;
//    
//    defaultParameters.dynamicsSpreadingCenter = 1.0;
//    defaultParameters.dynamicsSpreadingRange  = 0.3;
//    
//    defaultParameters.scalingsSpreadingCenter = 1.0;
//    defaultParameters.scalingsSpreadingRange  = 0.3;
//    
//    defaultParameters.rotationsSpreadingCenter = 0.0;
//    defaultParameters.rotationsSpreadingRange  = 0.0;
//    
//    tolerancesetmanually = false;
//    
//    setup(_config,  defaultParameters);
//    
//}
//
////--------------------------------------------------------------
//void GVF::setup(GVFConfig _config, GVFParameters _parameters)
//{
//    clear(); // just in case
//    // Set configuration and parameters
//    config      = _config;
//    parameters  = _parameters;
//    // Init random generators
//    normgen = std::mt19937(rd());
//    rndnorm = new std::normal_distribution<float>(0.0,1.0);
//    unifgen = std::default_random_engine(rd());
//    rndunif = new std::uniform_real_distribution<float>(0.0,1.0);
//}

//--------------------------------------------------------------
GVF::~GVF()
{
    if (rndnorm != NULL)
        delete (rndnorm);
    clear(); // not really necessary but it's polite ;)
}

//--------------------------------------------------------------
void GVF::clear()
{
    state = STATE_CLEAR;
    gestureTemplates.clear();
    mostProbableIndex = -1;
}

//--------------------------------------------------------------
void GVF::startGesture()
{
    if (state==STATE_FOLLOWING)
    {
        restart();
    }
    else if (state==STATE_LEARNING)
    {
        if (theGesture.getNumberOfTemplates()>0)
        {
            if (theGesture.getTemplateLength()>0)
                addGestureTemplate(theGesture);
        }
        theGesture.clear();
    }
}

//--------------------------------------------------------------
void GVF::addObservation(vector<float> data)
{
    theGesture.addObservation(data);
}

//--------------------------------------------------------------
void GVF::addGestureTemplate(GVFGesture & gestureTemplate)
{
    
    //    if (getState() != GVF::STATE_LEARNING)
    //        setState(GVF::STATE_LEARNING);
    
    int inputDimension = gestureTemplate.getNumberDimensions();
    config.inputDimensions = inputDimension;
    
    gestureTemplates.push_back(gestureTemplate);
    activeGestures.push_back(gestureTemplates.size());
    
    if(minRange.size() == 0){
        minRange.resize(inputDimension);
        maxRange.resize(inputDimension);
    }
    
    for(int j = 0; j < inputDimension; j++){
        minRange[j] = INFINITY;
        maxRange[j] = -INFINITY;
    }
    
    // compute min/max from the data
    for(int i = 0; i < gestureTemplates.size(); i++){
        GVFGesture& tGestureTemplate = gestureTemplates[i];
        vector<float>& tMinRange = tGestureTemplate.getMinRange();
        vector<float>& tMaxRange = tGestureTemplate.getMaxRange();
        for(int j = 0; j < inputDimension; j++){
            if(tMinRange[j] < minRange[j]) minRange[j] = tMinRange[j];
            if(tMaxRange[j] > maxRange[j]) maxRange[j] = tMaxRange[j];
        }
    }
    
    for(int i = 0; i < gestureTemplates.size(); i++){
        GVFGesture& tGestureTemplate = gestureTemplates[i];
        tGestureTemplate.setMinRange(minRange);
        tGestureTemplate.setMaxRange(maxRange);
    }
    train();
    
}

//--------------------------------------------------------------
void GVF::replaceGestureTemplate(GVFGesture & gestureTemplate, int index)
{
    if(gestureTemplate.getNumberDimensions()!=config.inputDimensions)
        return;
    if(minRange.size() == 0)
    {
        minRange.resize(config.inputDimensions);
        maxRange.resize(config.inputDimensions);
    }
    for(int j = 0; j < config.inputDimensions; j++)
    {
        minRange[j] = INFINITY;
        maxRange[j] = -INFINITY;
    }
    if (index<=gestureTemplates.size())
        gestureTemplates[index-1]=gestureTemplate;
    for(int i = 0; i < gestureTemplates.size(); i++)
    {
        GVFGesture& tGestureTemplate = gestureTemplates[i];
        vector<float>& tMinRange = tGestureTemplate.getMinRange();
        vector<float>& tMaxRange = tGestureTemplate.getMaxRange();
        for(int j = 0; j < config.inputDimensions; j++){
            if(tMinRange[j] < minRange[j]) minRange[j] = tMinRange[j];
            if(tMaxRange[j] > maxRange[j]) maxRange[j] = tMaxRange[j];
        }
    }
    for(int i = 0; i < gestureTemplates.size(); i++)
    {
        GVFGesture& tGestureTemplate = gestureTemplates[i];
        tGestureTemplate.setMinRange(minRange);
        tGestureTemplate.setMaxRange(maxRange);
    }
}

////--------------------------------------------------------------
//vector<float>& GVF::getGestureTemplateSample(int gestureIndex, float cursor)
//{
//    int frameindex = min((int)(gestureTemplates[gestureIndex].getTemplateLength() - 1),
//                         (int)(floor(cursor * gestureTemplates[gestureIndex].getTemplateLength() ) ) );
//    return gestureTemplates[gestureIndex].getTemplate()[frameindex];
//}

//--------------------------------------------------------------
GVFGesture & GVF::getGestureTemplate(int index){
    assert(index < gestureTemplates.size());
    return gestureTemplates[index];
}

//--------------------------------------------------------------
vector<GVFGesture> & GVF::getAllGestureTemplates(){
    return gestureTemplates;
}

//--------------------------------------------------------------
int GVF::getNumberOfGestureTemplates(){
    return (int)gestureTemplates.size();
}

//--------------------------------------------------------------
void GVF::removeGestureTemplate(int index){
    assert(index < gestureTemplates.size());
    gestureTemplates.erase(gestureTemplates.begin() + index);
}

//--------------------------------------------------------------
void GVF::removeAllGestureTemplates(){
    gestureTemplates.clear();
}

//----------------------------------------------
void GVF::train(){
    
    if (gestureTemplates.size() > 0)
    {
        
        // get the number of dimension in templates
        config.inputDimensions = gestureTemplates[0].getTemplateDimension();
        
        dynamicsDim = 2;    // hard coded: just speed now
        scalingsDim = config.inputDimensions;
        
        // manage orientation
        if (config.inputDimensions==2) rotationsDim=1;
        else if (config.inputDimensions==3) rotationsDim=3;
        else rotationsDim=0;
        
        // Init state space
        initVec(classes, parameters.numberParticles);                           // Vector of gesture class
        initVec(alignment, parameters.numberParticles);                         // Vector of phase values (alignment)
        initMat(dynamics, parameters.numberParticles, dynamicsDim);             // Matric of dynamics
        initMat(scalings, parameters.numberParticles, scalingsDim);             // Matrix of scaling
        if (rotationsDim!=0) initMat(rotations, parameters.numberParticles, rotationsDim);             // Matrix of rotations
        initMat(offsets, parameters.numberParticles, config.inputDimensions);
        initVec(weights, parameters.numberParticles);                           // Weights
        
        initMat(particles, parameters.numberParticles, 3);
        //            std::cout << particles.size() << " "  << parameters.numberParticles << std::endl;
        
        // bayesian elements
        initVec(prior, parameters.numberParticles);
        initVec(posterior, parameters.numberParticles);
        initVec(likelihood, parameters.numberParticles);
        
        
        initPrior();            // prior on init state values
        initNoiseParameters();  // init noise parameters (transition and likelihood)
        
        
        // weighted dimensions in case: default is not weighted
        if (parameters.dimWeights.size()!=config.inputDimensions){
            parameters.dimWeights = vector<float> (config.inputDimensions);
            for(int k = 0; k < config.inputDimensions; k++) parameters.dimWeights[k] = 1.0 / config.inputDimensions;
        }
        
        // NORMALIZATION
//        if (config.normalization) {     // update the global normaliation factor
//            globalNormalizationFactor = -1.0;
//            // loop on previous gestures already learned
//            // take the max of all the gesture learned ...
//            for (int k=0; k<getNumberOfGestureTemplates() ; k++){
//                for(int j = 0; j < config.inputDimensions; j++){
//                    float rangetmp = fabs(getGestureTemplate(k).getMaxRange()[j]-getGestureTemplate(k).getMinRange()[j]);
//                    if (rangetmp > globalNormalizationFactor)
//                        globalNormalizationFactor=rangetmp;
//                }
//            }
//        }
//        // only for logs
//        if (config.logOn) {
//            vecRef = vector<vector<float> > (parameters.numberParticles);
//            vecObs = vector<float> (config.inputDimensions);
//            stateNoiseDist = vector<float> (parameters.numberParticles);
//        }
    }
}

//--------------------------------------------------------------
//void GVF::initPrior()
//{
//    
//    // PATICLE FILTERING
//    for (int k = 0; k < parameters.numberParticles; k++)
//    {
//        initPrior(k);
//        
//        classes[k] = activeGestures[k % activeGestures.size()] - 1;
//    }
//    
//}

//--------------------------------------------------------------
void GVF::initPrior() //int pf_n)
{
    for (int pf_n = 0; pf_n < parameters.numberParticles; pf_n++)
    {
    // alignment
    alignment[pf_n] = ((*rndunif)(unifgen) - 0.5) * parameters.alignmentSpreadingRange + parameters.alignmentSpreadingCenter;    // spread phase
    
    
    // dynamics
    dynamics[pf_n][0] = ((*rndunif)(unifgen) - 0.5) * parameters.dynamicsSpreadingRange + parameters.dynamicsSpreadingCenter; // spread speed
    if (dynamics[pf_n].size()>1)
    {
        dynamics[pf_n][1] = ((*rndunif)(unifgen) - 0.5) * parameters.dynamicsSpreadingRange; // spread accel
    }
    
    // scalings
    for(int l = 0; l < scalings[pf_n].size(); l++) {
        scalings[pf_n][l] = ((*rndunif)(unifgen) - 0.5) * parameters.scalingsSpreadingRange + parameters.scalingsSpreadingCenter; // spread scalings
    }
    
    // rotations
    if (rotationsDim!=0)
        for(int l = 0; l < rotations[pf_n].size(); l++)
            rotations[pf_n][l] = ((*rndunif)(unifgen) - 0.5) * parameters.rotationsSpreadingRange + parameters.rotationsSpreadingCenter;    // spread rotations
    
    if (config.translate) for(int l = 0; l < offsets[pf_n].size(); l++) offsets[pf_n][l] = 0.0;
    
    
    prior[pf_n] = 1.0 / (float) parameters.numberParticles;
    
    // set the posterior to the prior at the initialization
    posterior[pf_n] = prior[pf_n];
        
                classes[pf_n] = activeGestures[pf_n % activeGestures.size()] - 1;
    }
    
}

//--------------------------------------------------------------
void GVF::initNoiseParameters() {
    
    // NOISE (ADDITIVE GAUSSIAN NOISE)
    // ---------------------------
    
    if (parameters.dynamicsVariance.size() != dynamicsDim)
    {
        float variance = parameters.dynamicsVariance[0];
        parameters.dynamicsVariance.resize(dynamicsDim);
        for (int k=0; k<dynamicsDim; k++)
            parameters.dynamicsVariance[k] = variance;
    }
    
    if (parameters.scalingsVariance.size() != scalingsDim)
    {
        float variance = parameters.scalingsVariance[0];
        parameters.scalingsVariance.resize(scalingsDim);
        for (int k=0; k<scalingsDim; k++)
            parameters.scalingsVariance[k] = variance;
    }
    
    if (rotationsDim!=0)
    {
        if (parameters.rotationsVariance.size() != rotationsDim)
        {
            float variance = parameters.rotationsVariance[0];
            parameters.rotationsVariance.resize(rotationsDim);
            for (int k=0; k<rotationsDim; k++)
                parameters.rotationsVariance[k] = variance;
        }
    }
    
    // ADAPTATION OF THE TOLERANCE IF DEFAULT PARAMTERS
    // ---------------------------
    if (!tolerancesetmanually){
        float obsMeanRange = 0.0f;
        for (int gt=0; gt<gestureTemplates.size(); gt++) {
            for (int d=0; d<config.inputDimensions; d++)
                obsMeanRange += (gestureTemplates[gt].getMaxRange()[d] - gestureTemplates[gt].getMinRange()[d])
                /config.inputDimensions;
        }
        obsMeanRange /= gestureTemplates.size();
        parameters.tolerance = obsMeanRange / 4.0f;  // dividing by an heuristic factor [to be learned?]
    }
}

//--------------------------------------------------------------
void GVF::setState(GVFState _state, vector<int> indexes)
{
    switch (_state)
    {
        case STATE_CLEAR:
            clear();
            theGesture.clear();
            break;
            
        case STATE_LEARNING:
            if ((state==STATE_LEARNING) && (theGesture.getNumberOfTemplates()>0))
            {
                if (learningGesture==-1)
                    addGestureTemplate(theGesture);
                else
                {
                    replaceGestureTemplate(theGesture, learningGesture);
                    learningGesture=-1;
                }
                if (indexes.size()!=0)
                    learningGesture=indexes[0];
            }
            state = _state;
            theGesture.clear();
            break;
            
        case STATE_FOLLOWING:
            if ((state==STATE_LEARNING) && (theGesture.getNumberOfTemplates()>0))
            {
                if (learningGesture==-1)
                    addGestureTemplate(theGesture);
                else
                {
                    replaceGestureTemplate(theGesture, learningGesture);
                    learningGesture=-1;
                }
            }
            if (gestureTemplates.size() > 0)
            {
                train();
                state = _state;
            }
            else
                state = STATE_CLEAR;
            theGesture.clear();
            break;
            
        default:
            theGesture.clear();
            break;
    }
}

//--------------------------------------------------------------
GVF::GVFState GVF::getState()
{
    return state;
}

////--------------------------------------------------------------
//int GVF::getDynamicsDimension(){
//    return dynamicsDim;
//}

//--------------------------------------------------------------
vector<int> GVF::getGestureClasses()
{
    return classes;
}

////--------------------------------------------------------------
//vector<float> GVF::getAlignment(){
//    return alignment;
//}
//
////--------------------------------------------------------------
//vector<float> GVF::getEstimatedAlignment(){
//    return estimatedAlignment;
//}
//
////--------------------------------------------------------------
//vector< vector<float> > GVF::getDynamics(){
//    return dynamics;
//}
//
////--------------------------------------------------------------
//vector< vector<float> > GVF::getEstimatedDynamics(){
//    return estimatedDynamics;
//}
//
////--------------------------------------------------------------
//vector< vector<float> > GVF::getScalings(){
//    return scalings;
//}
//
////--------------------------------------------------------------
//vector< vector<float> > GVF::getEstimatedScalings(){
//    return estimatedScalings;
//}
//
////--------------------------------------------------------------
//vector< vector<float> > GVF::getRotations(){
//    return rotations;
//}
//
////--------------------------------------------------------------
//vector< vector<float> > GVF::getEstimatedRotations(){
//    return estimatedRotations;
//}

////--------------------------------------------------------------
//vector<float> GVF::getEstimatedProbabilities(){
//    return estimatedProbabilities;
//}
//
////--------------------------------------------------------------
//vector<float> GVF::getEstimatedLikelihoods(){
//    return estimatedLikelihoods;
//}
//
////--------------------------------------------------------------
//vector<float> GVF::getWeights(){
//    return weights;
//}
//
////--------------------------------------------------------------
//vector<float> GVF::getPrior(){
//    return prior;
//}

////--------------------------------------------------------------
//vector<vector<float> > GVF::getVecRef() {
//    return vecRef;
//}
//
////--------------------------------------------------------------
//vector<float> GVF::getVecObs() {
//    return vecObs;
//}
//
////--------------------------------------------------------------
//vector<float> GVF::getStateNoiseDist(){
//    return stateNoiseDist;
//}

////--------------------------------------------------------------
//int GVF::getScalingsDim(){
//    return scalingsDim;
//}
//
////--------------------------------------------------------------
//int GVF::getRotationsDim(){
//    return rotationsDim;
//}

//--------------------------------------------------------------
void GVF::restart()
{
    theGesture.clear();
    initPrior();
}

#pragma mark - PARTICLE FILTERING

//--------------------------------------------------------------
void GVF::updatePrior(int n) {
    //cout << "\n updatePrior ...\n";
	//cout << "Gesture Template : " << gvf->getNumberOfGestureTemplates();
    // Update alignment / dynamics / scalings
    float L = gestureTemplates[classes[n]].getTemplateLength();
//cout << "\n gesture Templates creato ...\n";
    alignment[n] += (*rndnorm)(normgen) * parameters.alignmentVariance + dynamics[n][0]/L; // + dynamics[n][1]/(L*L);
    //cout << "\n Prior... alignment ...\n";
    if (dynamics[n].size()>1){
        dynamics[n][0] += (*rndnorm)(normgen) * parameters.dynamicsVariance[0] + dynamics[n][1]/L;
        dynamics[n][1] += (*rndnorm)(normgen) * parameters.dynamicsVariance[1];
    }
    else {
        dynamics[n][0] += (*rndnorm)(normgen) * parameters.dynamicsVariance[0];
    }
    
    //    for(int l= 0; l < dynamics[n].size(); l++)  dynamics[n][l] += (*rndnorm)(normgen) * parameters.dynamicsVariance[l];
    for(int l= 0; l < scalings[n].size(); l++)  scalings[n][l] += (*rndnorm)(normgen) * parameters.scalingsVariance[l];
    if (rotationsDim!=0) for(int l= 0; l < rotations[n].size(); l++)  rotations[n][l] += (*rndnorm)(normgen) * parameters.rotationsVariance[l];
    
    // update prior (bayesian incremental inference)
    prior[n] = posterior[n];
}

//--------------------------------------------------------------
void GVF::updateLikelihood(vector<float> obs, int n)
{
	//cout << "\n likelihood ...\n";
//    if (config.normalization) for (int kk=0; kk<vobs.size(); kk++) vobs[kk] = vobs[kk] / globalNormalizationFactor;
    
    if(alignment[n] < 0.0)
    {
        alignment[n] = fabs(alignment[n]);  // re-spread at the beginning
//        if (config.segmentation)
//            classes[n]   = n % getNumberOfGestureTemplates();
    }
    else if(alignment[n] > 1.0)
    {
        if (config.segmentation)
        {
//            alignment[n] = fabs(1.0-alignment[n]); // re-spread at the beginning
            alignment[n] = fabs((*rndunif)(unifgen) * 0.5);    //
            classes[n]   = n % getNumberOfGestureTemplates();
            offsets[n]   = obs;
            // dynamics
            dynamics[n][0] = ((*rndunif)(unifgen) - 0.5) * parameters.dynamicsSpreadingRange + parameters.dynamicsSpreadingCenter; // spread speed
            if (dynamics[n].size()>1)
                dynamics[n][1] = ((*rndunif)(unifgen) - 0.5) * parameters.dynamicsSpreadingRange;
            // scalings
            for(int l = 0; l < scalings[n].size(); l++)
                scalings[n][l] = ((*rndunif)(unifgen) - 0.5) * parameters.scalingsSpreadingRange + parameters.scalingsSpreadingCenter; // spread scalings
            // rotations
            if (rotationsDim!=0)
                for(int l = 0; l < rotations[n].size(); l++)
                    rotations[n][l] = ((*rndunif)(unifgen) - 0.5) * parameters.rotationsSpreadingRange + parameters.rotationsSpreadingCenter;    // spread rotations
            // prior
            prior[n] = 1/(float)parameters.numberParticles;
        }
        else{
            alignment[n] = fabs(2.0-alignment[n]); // re-spread at the end
        }
    }
    
    vector<float> vobs(config.inputDimensions);
    setVec(vobs, obs);
    
    if (config.translate)
        for (int j=0; j < config.inputDimensions; j++)
            vobs[j] = vobs[j] - offsets[n][j];
    
    
    // take vref from template at the given alignment
    int gestureIndex = classes[n];
    float cursor = alignment[n];
    int frameindex = min((int)(gestureTemplates[gestureIndex].getTemplateLength() - 1),
                         (int)(floor(cursor * gestureTemplates[gestureIndex].getTemplateLength() ) ) );
//    return gestureTemplates[gestureIndex].getTemplate()[frameindex];
    vector<float> vref = gestureTemplates[gestureIndex].getTemplate()[frameindex];; //getGestureTemplateSample(classes[n], alignment[n]);
    
    // Apply scaling coefficients
    for (int k=0;k < config.inputDimensions; k++)
    {
//        if (config.normalization) vref[k] = vref[k] / globalNormalizationFactor;
        vref[k] *= scalings[n][k];
    }
    
    // Apply rotation coefficients
    if (config.inputDimensions==2) {
        float tmp0=vref[0]; float tmp1=vref[1];
        vref[0] = cos(rotations[n][0])*tmp0 - sin(rotations[n][0])*tmp1;
        vref[1] = sin(rotations[n][0])*tmp0 + cos(rotations[n][0])*tmp1;
    }
    else if (config.inputDimensions==3) {
        // Rotate template sample according to the estimated angles of rotations (3d)
        vector<vector< float> > RotMatrix = getRotationMatrix3d(rotations[n][0],rotations[n][1],rotations[n][2]);
        vref = multiplyMat(RotMatrix, vref);
    }
    
    // weighted euclidean distance
    float dist = distance_weightedEuclidean(vref,vobs,parameters.dimWeights);
    
    if(parameters.distribution == 0.0f){    // Gaussian distribution
        likelihood[n] = exp(- dist * 1 / (parameters.tolerance * parameters.tolerance));
    }
    else {            // Student's distribution
        likelihood[n] = pow(dist/parameters.distribution + 1, -parameters.distribution/2 - 1);    // dimension is 2 .. pay attention if editing]
    }
//    // if log on keep track on vref and vobs
//    if (config.logOn){
//        vecRef.push_back(vref);
//        vecObs = vobs;
//    }
}

//--------------------------------------------------------------
void GVF::updatePosterior(int n) {
	//cout << "\n Posterior ...\n";
    posterior[n]  = prior[n] * likelihood[n];
}

//--------------------------------------------------------------
GVFOutcomes & GVF::update(vector<float> & observation)
{
    
    if (state != GVF::STATE_FOLLOWING) setState(GVF::STATE_FOLLOWING);
    //cout << "\n Inserisceto in funzione update...\n";
    theGesture.addObservation(observation);
	//cout << "\n riuscito ad aggiungere il nuovo vettore...\n";
    vector<float> obs = theGesture.getLastObservation();
    //cout << "\n riuscito a creare il nuovo vettore obs...\n";
    //    std::cout << obs[0] << " " << obs[0] << " "
    //                << gestureTemplates[0].getTemplate()[20][0] << " " << gestureTemplates[0].getTemplate()[20][1] << " "
    //                << gestureTemplates[1].getTemplate()[20][0] << " " << gestureTemplates[1].getTemplate()[20][1] << std::endl;
    
    
    // for each particle: perform updates of state space / likelihood / prior (weights)
    float sumw = 0.0;
    for(int n = 0; n< parameters.numberParticles; n++)
    {
       //cout << "\n number particles ...\n";
        for (int m=0; m<parameters.predictionSteps; m++)
        {
	 //      cout << "\n number predictions steps ...\n";
            updatePrior(n);
            updateLikelihood(obs, n);
            updatePosterior(n);
        }
        
        sumw += posterior[n];   // sum posterior to normalise the distrib afterwards
        
        particles[n][0] = alignment[n];
        particles[n][1] = scalings[n][0];
        particles[n][2] = classes[n];
    }
    
    // normalize the weights and compute the resampling criterion
    float dotProdw = 0.0;
    for (int k = 0; k < parameters.numberParticles; k++){
        posterior[k] /= sumw;
        dotProdw   += posterior[k] * posterior[k];
    }
    // avoid degeneracy (no particles active, i.e. weight = 0) by resampling
    if( (1./dotProdw) < parameters.resamplingThreshold)
        resampleAccordingToWeights(obs);
    
    // estimate outcomes
    estimates();
    
    return outcomes;
    
}

//--------------------------------------------------------------
void GVF::resampleAccordingToWeights(vector<float> obs)
{
    // covennient
    int numOfPart = parameters.numberParticles;
    
    // cumulative dist
    vector<float>           c(numOfPart);
    
    // tmp matrices
    vector<int>             oldClasses;
    vector<float>           oldAlignment;
    vector< vector<float> > oldDynamics;
    vector< vector<float> > oldScalings;
    vector< vector<float> > oldRotations;
    
    setVec(oldClasses,   classes);
    setVec(oldAlignment, alignment);
    setMat(oldDynamics,  dynamics);
    setMat(oldScalings,  scalings);
    if (rotationsDim!=0) setMat(oldRotations, rotations);
    
    
    c[0] = 0;
    for(int i = 1; i < numOfPart; i++) c[i] = c[i-1] + posterior[i];
    
    
    float u0 = (*rndunif)(unifgen)/numOfPart;
    
    int i = 0;
    for (int j = 0; j < numOfPart; j++)
    {
        float uj = u0 + (j + 0.) / numOfPart;
        
        while (uj > c[i] && i < numOfPart - 1){
            i++;
        }
        
        classes[j]   = oldClasses[i];
        alignment[j] = oldAlignment[i];
        
        for (int l=0;l<dynamicsDim;l++)     dynamics[j][l] = oldDynamics[i][l];
        for (int l=0;l<scalingsDim;l++)     scalings[j][l] = oldScalings[i][l];
        if (rotationsDim!=0) for (int l=0;l<rotationsDim;l++) rotations[j][l] = oldRotations[i][l];
        
        // update posterior (partilces' weights)
        posterior[j] = 1.0/(float)numOfPart;
    }
    
}


//--------------------------------------------------------------
void GVF::estimates(){
    
    
    int numOfPart = parameters.numberParticles;
    vector<float> probabilityNormalisation(getNumberOfGestureTemplates());
    setVec(probabilityNormalisation, 0.0f, getNumberOfGestureTemplates());            // rows are gestures
    setVec(estimatedAlignment, 0.0f, getNumberOfGestureTemplates());            // rows are gestures
    setMat(estimatedDynamics,  0.0f, getNumberOfGestureTemplates(), dynamicsDim);  // rows are gestures, cols are features + probabilities
    setMat(estimatedScalings,  0.0f, getNumberOfGestureTemplates(), scalingsDim);   // rows are gestures, cols are features + probabilities
    if (rotationsDim!=0) setMat(estimatedRotations,  0.0f, getNumberOfGestureTemplates(), rotationsDim);   // ..
    setVec(estimatedProbabilities, 0.0f, getNumberOfGestureTemplates());            // rows are gestures
    setVec(estimatedLikelihoods, 0.0f, getNumberOfGestureTemplates());            // rows are gestures
    
    //    float sumposterior = 0.;
    
    for(int n = 0; n < numOfPart; n++)
    {
        probabilityNormalisation[classes[n]] += posterior[n];
    }
    
    
    // compute the estimated features and likelihoods
    for(int n = 0; n < numOfPart; n++)
    {
        
        //        sumposterior += posterior[n];
        estimatedAlignment[classes[n]] += alignment[n] * posterior[n];
        
        for(int m = 0; m < dynamicsDim; m++)
            estimatedDynamics[classes[n]][m] += dynamics[n][m] * (posterior[n]/probabilityNormalisation[classes[n]]);
        
        for(int m = 0; m < scalingsDim; m++)
            estimatedScalings[classes[n]][m] += scalings[n][m] * (posterior[n]/probabilityNormalisation[classes[n]]);
        
        if (rotationsDim!=0)
            for(int m = 0; m < rotationsDim; m++)
                estimatedRotations[classes[n]][m] += rotations[n][m] * (posterior[n]/probabilityNormalisation[classes[n]]);
        
        if (!isnan(posterior[n]))
            estimatedProbabilities[classes[n]] += posterior[n];
        estimatedLikelihoods[classes[n]] += likelihood[n];
    }
    
    // calculate most probable index during scaling...
    float maxProbability = 0.0f;
    mostProbableIndex = -1;
    
    for(int gi = 0; gi < getNumberOfGestureTemplates(); gi++)
    {
        if(estimatedProbabilities[gi] > maxProbability){
            maxProbability      = estimatedProbabilities[gi];
            mostProbableIndex   = gi;
        }
    }
    //    std::cout << estimatedProbabilities[0] << " " << estimatedProbabilities[1] << std::endl;
    
    //    outcomes.estimations.clear();
    outcomes.likelihoods.clear();
    outcomes.alignments.clear();
    outcomes.scalings.clear();
    outcomes.dynamics.clear();
    outcomes.rotations.clear();
    
    // most probable gesture index
    outcomes.likeliestGesture = mostProbableIndex;
    
    // Fill estimation for each gesture
    for (int gi = 0; gi < gestureTemplates.size(); ++gi) {
        
        //        GVFEstimation estimation;
        outcomes.likelihoods.push_back(estimatedProbabilities[gi]);
        outcomes.alignments.push_back(estimatedAlignment[gi]);
        //        estimation.probability = estimatedProbabilities[gi];
        //        estimation.alignment   = estimatedAlignment[gi];
        
        
        vector<float> gDynamics(dynamicsDim, 0.0);
        for (int j = 0; j < dynamicsDim; ++j) gDynamics[j] = estimatedDynamics[gi][j];
        outcomes.dynamics.push_back(gDynamics);
        
        vector<float> gScalings(scalingsDim, 0.0);
        for (int j = 0; j < scalingsDim; ++j) gScalings[j] = estimatedScalings[gi][j];
        outcomes.scalings.push_back(gScalings);
        
        vector<float> gRotations;
        if (rotationsDim!=0)
        {
            gRotations.resize(rotationsDim);
            for (int j = 0; j < rotationsDim; ++j) gRotations[j] = estimatedRotations[gi][j];
            outcomes.rotations.push_back(gRotations);
        }
        
        //        estimation.likelihood = estimatedLikelihoods[gi];
        
        // push estimation for gesture gi in outcomes
        //        outcomes.estimations.push_back(estimation);
    }
    
    
    //    assert(outcomes.estimations.size() == gestureTemplates.size());
    
}

////--------------------------------------------------------------
//int GVF::getMostProbableGestureIndex()
//{
//    return mostProbableIndex;
//}

////--------------------------------------------------------------
//GVFOutcomes GVF::getOutcomes()
//{
//    return outcomes;
//}

////--------------------------------------------------------------
//GVFEstimation GVF::getTemplateRecogInfo(int templateNumber)
//{
//    if (getOutcomes().estimations.size() <= templateNumber) {
//        GVFEstimation estimation;
//        return estimation; // blank
//    }
//    else
//        return getOutcomes().estimations[templateNumber];
//}
//
////--------------------------------------------------------------
//GVFEstimation GVF::getRecogInfoOfMostProbable() // FIXME: Rename!
//{
//    int indexMostProbable = getMostProbableGestureIndex();
//
//    if ((getState() == GVF::STATE_FOLLOWING) && (getMostProbableGestureIndex() != -1)) {
//        return getTemplateRecogInfo(indexMostProbable);
//    }
//    else {
//        GVFEstimation estimation;
//        return estimation; // blank
//    }
//}


////--------------------------------------------------------------
//vector<float> & GVF::getGestureProbabilities()
//{
//    gestureProbabilities.resize(getNumberOfGestureTemplates());
//    setVec(gestureProbabilities, 0.0f);
//    for(int n = 0; n < parameters.numberParticles; n++)
//        gestureProbabilities[classes[n]] += posterior[n];
//    
//    return gestureProbabilities;
//}

//--------------------------------------------------------------
const vector<vector<float> > & GVF::getParticlesPositions(){
    return particles;
}

////--------------------------------------------------------------
//void GVF::setParameters(GVFParameters _parameters){
//    
//    // if the number of particles has changed, we have to re-allocate matrices
//    if (_parameters.numberParticles != parameters.numberParticles)
//    {
//        parameters = _parameters;
//        
//        // minimum number of particles allowed
//        if (parameters.numberParticles < 4) parameters.numberParticles = 4;
//        
//        // re-learn
//        train();
//        
//        // adapt the resampling threshold in case if RT < NS
//        if (parameters.numberParticles <= parameters.resamplingThreshold)
//            parameters.resamplingThreshold = parameters.numberParticles / 4;
//        
//    }
//    else
//        parameters = _parameters;
//    
//    
//}
//
//GVFParameters GVF::getParameters(){
//    return parameters;
//}

//--------------------------------------------------------------
// Update the number of particles
void GVF::setNumberOfParticles(int numberOfParticles){
    
    parameters.numberParticles = numberOfParticles;
    
    if (parameters.numberParticles < 4)     // minimum number of particles allowed
        parameters.numberParticles = 4;
    
    train();
    
    if (parameters.numberParticles <= parameters.resamplingThreshold) {
        parameters.resamplingThreshold = parameters.numberParticles / 4;
    }
    
}

//--------------------------------------------------------------
int GVF::getNumberOfParticles(){
    return parameters.numberParticles; // Return the number of particles
}

//--------------------------------------------------------------
void GVF::setActiveGestures(vector<int> activeGestureIds)
{
    int argmax = *std::max_element(activeGestureIds.begin(), activeGestureIds.end());
    if (activeGestureIds[argmax] <= gestureTemplates.size())
    {
        activeGestures = activeGestureIds;
    }
    else
    {
        activeGestures.resize(gestureTemplates.size());
        std::iota(activeGestures.begin(), activeGestures.end(), 1);
    }
}

//--------------------------------------------------------------
void GVF::setPredictionSteps(int predictionSteps)
{
    if (predictionSteps<1)
        parameters.predictionSteps = 1;
    else
        parameters.predictionSteps = predictionSteps;
}

//--------------------------------------------------------------
int GVF::getPredictionSteps()
{
    return parameters.predictionSteps; // Return the number of particles
}

//--------------------------------------------------------------
// Update the resampling threshold used to avoid degeneracy problem
void GVF::setResamplingThreshold(int _resamplingThreshold){
    if (_resamplingThreshold >= parameters.numberParticles)
        _resamplingThreshold = floor(parameters.numberParticles/2.0f);
    parameters.resamplingThreshold = _resamplingThreshold;
}

//--------------------------------------------------------------
// Return the resampling threshold used to avoid degeneracy problem
int GVF::getResamplingThreshold(){
    return parameters.resamplingThreshold;
}

//--------------------------------------------------------------
// Update the standard deviation of the observation distribution
// this value acts as a tolerance for the algorithm
// low value: less tolerant so more precise but can diverge
// high value: more tolerant so less precise but converge more easily
void GVF::setTolerance(float _tolerance){
    if (_tolerance <= 0.0) _tolerance = 0.1;
    parameters.tolerance = _tolerance;
    tolerancesetmanually = true;
}

//--------------------------------------------------------------
float GVF::getTolerance(){
    return parameters.tolerance;
}

////--------------------------------------------------------------
void GVF::setDistribution(float _distribution){
    //nu = _distribution;
    parameters.distribution = _distribution;
}
//
////--------------------------------------------------------------
//float GVF::getDistribution(){
//    return parameters.distribution;
//}

//void GVF::setDimWeights(vector<float> dimWeights){
//    if (dimWeights.size()!=parameters.dimWeights.size())
//        parameters.dimWeights.resize(dimWeights.size());
//    parameters.dimWeights = dimWeights;
//}
//
//vector<float> GVF::getDimWeights(){
//    return parameters.dimWeights;
//}


//// VARIANCE COEFFICIENTS: PHASE
////--------------------------------------------------------------
//void GVF::setAlignmentVariance(float alignmentVariance){
//    parameters.alignmentVariance = sqrt(alignmentVariance);
//}
////--------------------------------------------------------------
//float GVF::getAlignmentVariance(){
//    return parameters.alignmentVariance;
//}


// VARIANCE COEFFICIENTS: DYNAMICS
//--------------------------------------------------------------
//void GVF::setDynamicsVariance(float dynVariance)
//{
//    for (int k=0; k< parameters.dynamicsVariance.size(); k++)
//        parameters.dynamicsVariance[k] = dynVariance;
//}
//--------------------------------------------------------------
void GVF::setDynamicsVariance(float dynVariance, int dim)
{
    if (dim == -1)
    {
        for (int k=0; k< parameters.dynamicsVariance.size(); k++)
            parameters.dynamicsVariance[k] = dynVariance;
    }
    else
    {
        if (dim<parameters.dynamicsVariance.size())
            parameters.dynamicsVariance[dim-1] = dynVariance;
    }
}

//--------------------------------------------------------------
void GVF::setDynamicsVariance(vector<float> dynVariance)
{
    parameters.dynamicsVariance = dynVariance;
}
//--------------------------------------------------------------
vector<float> GVF::getDynamicsVariance()
{
    return parameters.dynamicsVariance;
}

//--------------------------------------------------------------
void GVF::setScalingsVariance(float scaleVariance, int dim)
{
    if (dim == -1)
    {
        for (int k=0; k< parameters.scalingsVariance.size(); k++)
            parameters.scalingsVariance[k] = scaleVariance;
    }
    else
    {
        if (dim<parameters.scalingsVariance.size())
            parameters.scalingsVariance[dim-1] = scaleVariance;
    }
}

//--------------------------------------------------------------
void GVF::setScalingsVariance(vector<float> scaleVariance)
{
    parameters.scaleVariance = scaleVariance;
}

//--------------------------------------------------------------
vector<float> GVF::getScalingsVariance()
{
    return parameters.scalingsVariance;
}

//--------------------------------------------------------------
void GVF::setRotationsVariance(float rotationVariance, int dim)
{
    if (dim == -1)
    {
        for (int k=0; k< parameters.rotationsVariance.size(); k++)
            parameters.rotationsVariance[k] = rotationVariance;
    }
    else
    {
        if (dim<parameters.rotationsVariance.size())
            parameters.scalingsVariance[dim-1] = rotationVariance;
    }
}

//--------------------------------------------------------------
void GVF::setRotationsVariance(vector<float> rotationVariance)
{
    parameters.scaleVariance = rotationVariance;
}

//--------------------------------------------------------------
vector<float> GVF::getRotationsVariance()
{
    return parameters.rotationsVariance;
}

//--------------------------------------------------------------
void GVF::setSpreadDynamics(float center, float range, int dim)
{
    parameters.dynamicsSpreadingCenter = center;
    parameters.dynamicsSpreadingRange = range;
}

//--------------------------------------------------------------
void GVF::setSpreadScalings(float center, float range, int dim)
{
    parameters.scalingsSpreadingCenter = center;
    parameters.scalingsSpreadingRange = range;
}

//--------------------------------------------------------------
void GVF::setSpreadRotations(float center, float range, int dim)
{
    parameters.rotationsSpreadingCenter = center;
    parameters.rotationsSpreadingRange = range;
}

//--------------------------------------------------------------
void GVF::translate(bool translateFlag)
{
    config.translate = translateFlag;
}

//--------------------------------------------------------------
void GVF::segmentation(bool segmentationFlag)
{
    config.segmentation = segmentationFlag;
}


// UTILITIES

//--------------------------------------------------------------
// Save function. This function is used by applications to save the
// vocabulary in a text file given by filename (filename is also the complete path + filename)
void GVF::saveTemplates(string filename){
    
    std::string directory = filename;
    
    std::ofstream file_write(directory.c_str());
    
    for(int i=0; i < gestureTemplates.size(); i++) // Number of gesture templates
    {
        file_write << "template " << i << " " << config.inputDimensions << endl;
        vector<vector<float> > templateTmp = gestureTemplates[i].getTemplate();
        for(int j = 0; j < templateTmp.size(); j++)
        {
            for(int k = 0; k < config.inputDimensions; k++)
                file_write << templateTmp[j][k] << " ";
            file_write << endl;
        }
    }
    file_write.close();
    
}




//--------------------------------------------------------------
// Load function. This function is used by applications to load a vocabulary
// given by filename (filename is also the complete path + filename)
void GVF::loadTemplates(string filename){
    //    clear();
    //
    
    GVFGesture loadedGesture;
    loadedGesture.clear();
    
    ifstream infile;
    stringstream doung;
    
    infile.open (filename.c_str(), ifstream::in);
    //
    string line;
    vector<string> list;
    int cl = -1;
    while(!infile.eof())
    {
        cl++;
        infile >> line;
        
        list.push_back(line);
    }
    
    int k = 0;
    int template_id = -1;
    int template_dim = 0;
    
    
    while (k < (list.size() - 1)){ // TODO to be changed if dim>2
        
        
        if (!strcmp(list[k].c_str(),"template"))
        {
            template_id = atoi(list[k+1].c_str());
            template_dim = atoi(list[k+2].c_str());
            k = k + 3;
            
            if (loadedGesture.getNumberOfTemplates() > 0){
                addGestureTemplate(loadedGesture);
                loadedGesture.clear();
            }
        }
        
        if (template_dim <= 0){
            //post("bug dim = -1");
        }
        else{
            
            vector<float> vect(template_dim);
            
            for (int kk = 0; kk < template_dim; kk++)
                vect[kk] = (float) atof(list[k + kk].c_str());
            
            loadedGesture.addObservation(vect);
        }
        k += template_dim;
        
    }
    
    if (loadedGesture.getTemplateLength() > 0){
        addGestureTemplate(loadedGesture);
        loadedGesture.clear();
    }
    
    infile.close();
}



