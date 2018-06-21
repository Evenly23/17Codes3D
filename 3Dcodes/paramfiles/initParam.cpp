#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

istream *input;
ofstream output;
ifstream infile;
istringstream inString;

void run();

int main()
{	
	output.open("namedParam.h");
	output << "#ifndef PPP_H" << endl
		<< "#define PPP_H" << endl << endl

		<< "#include <map>" << endl
		<< "#include <string>" << endl
		<< "using namespace std;" << endl << endl

		<< "void makeNamedParams(map<string, string> &namedParams);" << endl
		<< "void makeNamedParams0(map<string, string> &namedParams);" << endl
		<< "void makeNamedParams1(map<string, string> &namedParams);" << endl
		<< "void makeNamedParams2(map<string, string> &namedParams);" << endl
		<< "void makeNamedParams3(map<string, string> &namedParams);" << endl
		<< "void makeNamedParams4(map<string, string> &namedParams);" << endl << endl

		<< "#endif" << endl;
	output.close();


	//start the include
	output.open("namedParam.cpp");
	output << "#include \"namedParam.h\"" << endl;
	output << endl;
	//end

	output << "void makeNamedParams(map<string, string> &namedParams){" << endl;
    infile.open("defaultParams.txt", ifstream::in);
    run();
    output << "}" << endl;

	//param0 start
	output << "void makeNamedParams0(map<string, string> &namedParams){" << endl;
    infile.open("defaultParams_t0.txt", ifstream::in);
    run();
    output << "}" << endl;
    
    output << "void makeNamedParams1(map<string, string> &namedParams){" << endl;
    infile.open("defaultParams_t1.txt", ifstream::in);
    run();
    output << "}" << endl;

    output << "void makeNamedParams2(map<string, string> &namedParams){" << endl;
    infile.open("defaultParams_t2.txt", ifstream::in);
    run();
    output << "}" << endl;

    output << "void makeNamedParams3(map<string, string> &namedParams){" << endl;
    infile.open("defaultParams_t3.txt", ifstream::in);
    run();
    output << "}" << endl;

    output << "void makeNamedParams4(map<string, string> &namedParams){" << endl;
    infile.open("defaultParams_t4.txt", ifstream::in);
    run();
    output << "}" << endl;

    output.close();
	return 0;
}

void run(){
    input = &(infile);

    string name;
    bool fBlockComment = false;
    while(!input->eof())
    {

        // Skip comments and empty lines
        std::string str;
        std::getline(*input, str);
        if (str.length() >= 2 && str.substr(0,2) == "/*") {
            fBlockComment = true;
        } else if (str == "*/") {
            fBlockComment = false;
        }
        
        if(fBlockComment || str == "" || str[0] == '#' ) {
            continue;
        }

        // otherwise parse strings
        stringstream s(str);
        std::string key;
        std::string value;
        std::getline(s, key, '\t');      //read thru tab
        std::getline(s, value);          //read thru newline
        if(value.empty()) {
            continue;
        }
        output << "    namedParams[\"" << key << "\"] = \"" << value << "\";" << endl;
    }

    infile.close();
}
