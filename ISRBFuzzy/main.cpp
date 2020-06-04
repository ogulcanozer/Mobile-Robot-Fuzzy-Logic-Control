/*_______________________________________________________________________________
# CE801 Intelligent Systems and Robotics | Ogulcan Ozer. | 11 December 2018
	UNFINISHED. Read the rulebase from a text file.
_______________________________________________________________________________*/
/*-------------------------------------------------------------------------------
#	CE801 Fuzzy logic system for right edge following and obstacle avoidance
	with context-based blending for ARIA Robot.

	Usage: Upload the executable to the robot and run.
-------------------------------------------------------------------------------*/

#include <iostream>
#include<iostream>
#include<fstream>
#include<string>
#include <vector>
#include <string>
#include "Aria.h"
using namespace std;

/*-------------------------------------------------------------------------------
#	Data and structs.
-------------------------------------------------------------------------------*/
//RuleBase for right and left sensors, output is reversed for left edge following.
static const vector<vector<string>> RuleBaseRF =
{
	{	//possible inputs
		"FAR,FAR",//1
		"FAR,NORM",//2
		"FAR,NEAR",//3
		"NORM,FAR",//4
		"NORM,NORM",//5
		"NORM,NEAR",//6
		"NEAR,FAR",//7
		"NEAR,NORM",//8
		"NEAR,NEAR"//9
	},
	{	//possible outputs
		"HIGH,LOW",//1
		"LOW,HIGH",//2
		"LOW,HIGH",//3
		"HIGH,MED",//4
		"HIGH,HIGH",//5
		"LOW,HIGH",//6
		"LOW,HIGH",//7
		"LOW,HIGH",//8
		"LOW,HIGH"//9
	}
};

//RuleBase for front sensors.
static const vector<vector<string>> RuleBaseOA =
{
	{
		//possible inputs
"NEAR,NEAR,NEAR",//1
"NEAR,NEAR,NORM",//2
"NEAR,NEAR,FAR",//3
"NEAR,NORM,NEAR",//4
"NEAR,NORM,NORM",//5
"NEAR,NORM,FAR",//6
"NEAR,FAR,NEAR",//7
"NEAR,FAR,NORM",//8
"NEAR,FAR,FAR",//9
"NORM,NEAR,NEAR",//10
"NORM,NEAR,NORM",//11
"NORM,NEAR,FAR",//12
"NORM,NORM,NEAR",//13
"NORM,NORM,NORM",//14
"NORM,NORM,FAR",//15
"NORM,FAR,NEAR",//16
"NORM,FAR,NORM",//17
"NORM,FAR,FAR",//18
"FAR,NEAR,NEAR",//19
"FAR,NEAR,NORM",//20
"FAR,NEAR,FAR",//21
"FAR,NORM,NEAR",//22
"FAR,NORM,NORM",//23
"FAR,NORM,FAR",//24
"FAR,FAR,NEAR",//25
"FAR,FAR,NORM",//26
"FAR,FAR,FAR"//27
	},
	{	//possible outputs
"LOW,HIGH",//28
"HIGH,LOW",//29
"HIGH,LOW",//30
"LOW,LOW",//31
"HIGH,LOW",//32
"HIGH,LOW",//33
"MED,MED",//34
"HIGH,LOW",//35
"HIGH,LOW",//36
"LOW,HIGH",//37
"LOW,HIGH",//38
"HIGH,LOW",//39
"LOW,HIGH",//40
"MED,HIGH",//41
"HIGH,MED",//42
"LOW,HIGH",//43
"HIGH,HIGH",//44
"HIGH,MED",//45
"LOW,HIGH",//46
"LOW,HIGH",//47
"LOW,HIGH",//48
"LOW,HIGH",//49
"MED,HIGH",//50
"MED,MED",//51
"LOW,HIGH",//52
"MED,MED",//53
"HIGH,HIGH"//54


	}
};


//Input struct to hold sensor readings their memberships.
struct Input
{
	vector<string> labels;
	double value;//Input from sonar.
	vector<double> memberships;//For possible memberships.
};

//Structure for holding a member of a Rule output.
struct OutRule
{
	string label;
	double value;
	double fire;


};

//Structure to hold a function's values for membership calculation.
//________________________________________________
//
//|     _____       |              |   ______
//|    /|   |\      |    /|\       |  |      |
//|   / |   | \     |   / | \      |  |      |
//|__/__|___|__\__  |__/__|__\__   |__|______|__
//  s  t1   t2  e     s  t1   e       s      e
//                       t2           t1     t2
//________________________________________________
struct Function
{
	string label;
	double start;
	double top[2];
	double end;

};

/*-------------------------------------------------------------------------------
#	Variables for fuzzy-sets
-------------------------------------------------------------------------------*/
static const Function d1 = { "LEFT", 0,{ 0, 451 }, 1400 };// Functions for context-based blending .
static const Function d2 = { "RIGHT", 0,{ 0, 1401 }, 5001 };//
static const Function d3 = { "FRONT", 0,{ 0 , 3000 }, 5001 };//

static const Function r0ne = { "NEAR", 0,{ 0, 200 }, 450 };//Functions for right edge following, both for sensor 7 and 6.
static const Function r0no = { "NORM", 350,{ 450, 600 }, 700 };//
static const Function r0f = { "FAR", 600,{ 850, 5001 }, 5001 };//
static const Function r1ne = { "NEAR", 0,{ 0, 450 }, 600 };//
static const Function r1no = { "NORM", 450,{ 550, 850 }, 950 };//
static const Function r1f = { "FAR",  900,{ 1200, 5001 }, 5001 };//


//Functions for obstacle avoidance, for sensors 2, minimum of 3-4 and 5.
static const Function f0ne = { "NEAR", 0,{ 0, 700 }, 900 };//
static const Function f0no = { "NORM", 700,{ 1100, 1600 }, 2000 };//
static const Function f0f = { "FAR",  1700,{ 2600, 5001 }, 5001 };//
static const Function f1ne = { "NEAR", 0,{ 0, 900 }, 1200 };//
static const Function f1no = { "NORM", 900,{ 1300, 1500 }, 1900 };//
static const Function f1f = { "FAR",  1600,{ 2200, 5001 }, 5001 };//
static const Function f2ne = { "NEAR", 0,{ 0, 700 }, 900 };//
static const Function f2no = { "NORM", 700,{ 1100, 1600 }, 2000 };//
static const Function f2f = { "FAR",  1700,{ 2600, 5001 }, 5001 };//


static const Function yl = { "LOW", 40,{ 90, 90 }, 140 };//Functions for motor speeds.
static const Function ym = { "MED", 120,{ 170, 170 }, 220 };//
static const Function yh = { "HIGH", 200,{ 250, 250 }, 300 };//

static const vector<Function> x0 = { r0ne,r0no,r0f };//Fuzzy sets for right edge sensors.
static const vector<Function> x1 = { r1ne,r1no,r1f };//

static const vector<Function> f0 = { f0ne,f0no,f0f };//Fuzzy sets for front sensors.
static const vector<Function> f1 = { f1ne,f1no,f1f };//
static const vector<Function> f2 = { f2ne,f2no,f2f };//

//static const vector<Function> y = { yl,ym,yh };//Fuzzy set for motor speed.

static const vector<Function> o1 = { d1 };//Wrapper sets for context based blending, since 
static const vector<Function> o2 = { d2 };//membership function only accepts Function vectors.
static const vector<Function> o3 = { d3 };//

 /*-------------------------------------------------------------------------------
 #	Function definitions
 -------------------------------------------------------------------------------*/
 //Permutation of activated set members for 2 input sets (Left edge and right edge following).
vector< vector<OutRule> > permutate2(static const vector<vector<string>> RuleBase, vector<Input> inputs);

//Permutation of activated set members for 3 input sets (Obstacle avoidance).
vector< vector<OutRule> > permutate3(vector<Input> inputs);

//Return the index of a rule in the rule base.
int ruleIndex(vector<vector<string>> rules, string s);

//Get output speeds for left and right wheel from fire strengths.
vector<double> getSpeeds(vector<vector<OutRule>> func);

//Get the membership value of each input.
double getDegree(Input *in, const vector<Function> *func);

//Set the firing strength of each output.
void setFire(vector<vector<OutRule>> &func);


/*-------------------------------------------------------------------------------
#	Main Program
-------------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	//Initialize the robot.
	Aria::init();
	ArRobot robot;
	ArPose pose;
	ArSensorReading *sonarSensor[8];
	//Parse command line args.
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot()) {
		std::cout << "Robot Connected !" << std::endl;
	}
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();
	//End of robot initialization.


	double minL, minR, minF;//For holding sensor minimums to be used in context blending.

	//Main loop.
	while (true)
	{
		double sonarRange[8];//Read sonar values.
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();

		}//
		

		cout << "Sensor R0:" << sonarRange[7] << endl;
		cout << "Sensor R1:" << sonarRange[6] << endl;
		cout << "Sensor F0:" << sonarRange[2] << endl;
		cout << "Sensor F1:" << min(sonarRange[3], sonarRange[4]) << endl;
		cout << "Sensor F2:" << sonarRange[5] << endl;
		minL = min(sonarRange[0], sonarRange[1]);//Get minimums for each context 
		minR = min(sonarRange[6], sonarRange[7]);//
		minF = min(min(sonarRange[5], sonarRange[2]), min(sonarRange[3], sonarRange[4]));//
		//Wrap in Input structs.
		Input oR = { {}, minR };/* The fixed problem was, instead of assigning minR to oR, minR was assigned to oL and vice versa. */
		Input oL = { {}, minL };
		Input oF = { {}, minF };
		//Get memberships.
		getDegree(&oL, &o1);
		getDegree(&oR, &o2);
		getDegree(&oF, &o3);
		//Wrap each sonar input.
		Input in1 = { {}, sonarRange[7] };
		Input in2 = { {}, sonarRange[6] };
		Input in3 = { {}, sonarRange[2] };
		Input in4 = { {}, min(sonarRange[3],sonarRange[4]) };
		Input in5 = { {}, sonarRange[5] };
		Input in6 = { {}, sonarRange[0] };
		Input in7 = { {}, sonarRange[1] };
		//Get membership values of each sonar.
		getDegree(&in1, &x0);
		getDegree(&in2, &x1);
		getDegree(&in3, &f0);
		getDegree(&in4, &f1);
		getDegree(&in5, &f2);
		getDegree(&in6, &x0);
		getDegree(&in7, &x1);
		

		//Group sensor outputs according to their use.
		vector<Input> inputsRE = { in1,in2 };//right
		vector<Input> inputsLE = { in6,in7 };//left
		vector<Input> inputsOA = { in3,in4,in5 };//obstacle avoidance

		vector<vector<OutRule>> outRE = permutate2(RuleBaseRF, inputsRE);//Get all the rules resulting from the input - for right edge
		vector<vector<OutRule>> outLE = permutate2(RuleBaseRF, inputsLE);//-left edge
		vector<vector<OutRule>> outOA = permutate3(inputsOA);//- obstacle avoidance
															

		//Set firing strengths of each group
		setFire(outLE);
		setFire(outOA);
		setFire(outRE);

		//Get speeds of the groups for each wheel
		vector<double>speedRE = getSpeeds(outRE);
		vector<double>speedOA = getSpeeds(outOA);
		vector<double>speedLE = getSpeeds(outLE);
		

		//Adjust speeds according to context-based blending.
		double lms = (((speedOA[0] * oF.memberships[0]) + (speedRE[0] * oR.memberships[0]) + (speedLE[1] * oL.memberships[0])) / (oF.memberships[0] + oL.memberships[0] + oR.memberships[0]));//Change the output of left
		double rms = (((speedOA[1] * oF.memberships[0]) + (speedRE[1] * oR.memberships[0]) + (speedLE[0] * oL.memberships[0])) / (oF.memberships[0] + oL.memberships[0] + oR.memberships[0]));//
																																												
		//Send them to the wheels.
		robot.setVel2(lms, rms);

		ArUtil::sleep(75);//Sleep

	}
	////End of main program.




}

/*-------------------------------------------------------------------------------
#	Functions
-------------------------------------------------------------------------------*/

//Permutation of activated set members for 2 input sets (Left edge and right edge following).
vector< vector<OutRule> > permutate2(static const vector<vector<string>> RuleBase, vector<Input> inputs) {

	vector< vector<OutRule >> output;//output to be returned.
	output.resize(2);
	OutRule temp;

	//Repeat 9 times.
	for (int i = 0; i < 3; i++) {

		for (int j = 0; j < 3; j++) {



			double m1 = inputs[0].memberships[i];
			double m2 = inputs[1].memberships[j];

			if ((m1 != 0) && (m2 != 0)) {//For each input, if memberships are not 0, get the corresponding rule.
				int idx = ruleIndex(RuleBase, inputs[0].labels[i] + "," + inputs[1].labels[j]);
				int a = RuleBase[1][idx].find(",", 0);
				temp.value = min(m1, m2);
				temp.label = (RuleBase[1][idx]).substr(0, a);
				output[0].push_back(temp);

				temp.label = (RuleBase[1][idx]).substr(a + 1);
				output[1].push_back(temp);

			}



		}

	}

	return output;



}

//Permutation of activated set members for 3 input sets (Obstacle avoidance).
vector< vector<OutRule> > permutate3(vector<Input> inputs) {

	vector< vector<OutRule >> output;//output to be returned.
	output.resize(2);
	OutRule temp;

	//Repeat 27 times.
	for (int i = 0; i < 3; i++) {

		for (int j = 0; j < 3; j++) {

			for (int k = 0; k < 3; k++) {


				double m1 = inputs[0].memberships[i];
				double m2 = inputs[1].memberships[j];
				double m3 = inputs[2].memberships[k];

				if ((m1 != 0) && (m2 != 0) && (m3 != 0)) {//For each input, if memberships are not 0, get the corresponding rule.
					int idx = ruleIndex(RuleBaseOA, inputs[0].labels[i] + "," + inputs[1].labels[j] + "," + inputs[1].labels[k]);
					int a = RuleBaseOA[1][idx].find(",", 0);
					double t = min(m1, m2);
					temp.value = min(t, m3);
					temp.label = (RuleBaseOA[1][idx]).substr(0, a);
					output[0].push_back(temp);
					temp.label = (RuleBaseOA[1][idx]).substr(a + 1);
					output[1].push_back(temp);



				}
			}


		}

	}

	return output;



}

//Return the index of a rule in the rule base.
int ruleIndex(vector<vector<string>> rules, string s)
{
	for (int i = 0; i < rules[0].size(); i++)
	{
		if (rules[0][i].compare(s) == 0) {
			return i;
		}
	}
	return -1;
}

//Get output speeds for left and right wheel from fire strengths.
vector<double> getSpeeds(vector<vector<OutRule>> func)
{
	double leftu = 0, leftd = 0, rightu = 0, rightd = 0;

	for (int i = 0; i < func[0].size(); i++) {//Do for left

		leftu = leftu + func[0][i].fire;
		leftd = leftd + func[0][i].value;
	}
	for (int i = 0; i < func[1].size(); i++) {//Do for right
		rightu = rightu + func[1][i].fire;
		rightd = rightd + func[1][i].value;
	}
	vector<double> speeds = { leftu / leftd,rightu / rightd };

	return speeds;
}

//Get the membership value of each input.
double getDegree(Input *in, const vector<Function> *func)
{

	int c = 0;
	for (auto tr : *func) {
		in->labels.push_back(tr.label);

		// This part checks for special cases.
		if ((in->value <= tr.start) || (in->value >= tr.end))//Check if the value is at the edges.
		{
			if (tr.start == in->value && tr.start == tr.top[0]) {
				in->memberships.push_back(1);//Step function, or edge steps up at the beginning
			}
			else if (tr.end == in->value && tr.end == tr.top[1]) {
				in->memberships.push_back(1);//Step function, or edge steps down at the end

			}
			else {
				in->memberships.push_back(0.0);//Outside of the function bounds
			}


		}
		else {

			if (tr.start < in->value && in->value < tr.top[0])
				in->memberships.push_back((in->value - tr.start) / (tr.top[0] - tr.start));//If rising edge.
			if (tr.top[0] <= in->value && in->value <= tr.top[1])
				in->memberships.push_back(1); //If highest point.
			if (tr.top[1] < in->value && in->value < tr.end)
				in->memberships.push_back((tr.end - in->value) / (tr.end - tr.top[1]));//If falling edge.
		}
	}
	return 0;

}

//Set the firing strength of each output for a given output rule.
void setFire(vector<vector<OutRule>> &func)
{
	for (int i = 0; i < func[0].size(); i++) {//Do for left wheel.
		if (func[0][i].label.compare("LOW") == 0) {
			func[0][i].fire = func[0][i].value*(yl.end + yl.start) / 2;
		}
		else if (func[0][i].label.compare("MED") == 0) {
			func[0][i].fire = func[0][i].value*(ym.end + ym.start) / 2;
		}
		else {
			func[0][i].fire = func[0][i].value*(yh.end + yh.start) / 2;
		}
	}
	for (int i = 0; i < func[1].size(); i++) {//Do for right wheel
		if (func[1][i].label.compare("LOW") == 0) {
			func[1][i].fire = func[1][i].value*(yl.end + yl.start) / 2;
		}
		else if (func[1][i].label.compare("MED") == 0) {
			func[1][i].fire = func[1][i].value*(ym.end + ym.start) / 2;
		}
		else {
			func[1][i].fire = func[1][i].value*(yh.end + yh.start) / 2;
		}
	}


}

/*-------------------------------------------------------------------------------
#	End of program
-------------------------------------------------------------------------------*/