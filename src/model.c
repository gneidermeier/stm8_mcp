//
// curve_gen;
//  generate a exponential decay function with paramters;
//  dump data to csv;
// 11/2020; G. Neidermeier;
//


// clear the Scilab workspace;

clear;


// args for Tau, A, and x/y offsets ????
TBLSZ = 180; // needs to be 250 but the dec2hex clips if the data goes negative :)
TBLMAX = TBLSZ - 1; // this constant is for generating indices colum e.g. [0:249]


// started with parameters from excel curve fit (A=922, TAU=50)
// sync range [32-4E] MAX=56
//A = 1500; // brings the idle (@DC=32) slow/high enuff to sync 
//A = 1800; // starts and runs close to time @ 12v on BPS
A = 1700; // 12.5v WW (@1800, timing is too inhibited for the 12v WW)
TAU = 50;
Y_INT = 0;
X_INT = 0;



//new table with 2 fields and 250 rows;
dtable(TBLSZ,2)=0;

// column 2 is "x" of the function ( will put behing comments in the C table);
dtable(:,2)=[0:TBLMAX];

// column 1 is the table value;

// between about 10-30%, an exponential decay function tracks better
// (1)  A * e ^ -t/Tau
dtable(:,1) =  Y_INT +  A * exp( -1 / TAU * ( X_INT + dtable(:,2) ) );

// the response seems to become more linear past 30%, this segment works up to about 57%
// (2)  Y = mx + b
SR=75;       // set starting row for linear segment [SR:TBLSZ]
OFFS=560;    // @ A=1500
OFFS=560+70; // @ A=1800
OFFS=560+60; // @ 1700  for the 12v WW

dtable(SR:TBLSZ,1) = ( 1 -  3 * ( dtable( SR:TBLSZ,2 )  + 0 )  + OFFS )
// plot( 1 -  3 * ( dtable( 1:TBLSZ,2 )  + 0 )  + 560, 'red' ) // use SR to offset on X-axis

// wip develop data in the range of the ramp segment (may be able to slow down slightly from the default ramp-to speed)
FR=30  // set ending row for linear segment [1:FR]
//dtable(1:FR,1) = ( 1 -  20 * ( dtable( 1:FR,2 )  + 0 )  + 1420 ) // maybe
//dtable(1:FR,1) = ( 1 -  70 * ( dtable( 1:FR,2 )  + 0 )  + 2900 )
//dtable(1:FR,1) = ( 1 -  100 * ( dtable( 1:FR,2 )  + 0 )  + 3750 )
// plot( 1 -  20 * ( dtable( 1:TBLSZ,2 )  + 0 )  + 0, 'r' ) 


// plot line for comparison
plot( 1 -  10 * ( dtable(:,2) + 60)  + 1700, 'cya.');


//new table with 3 fields and 250 rows;
itable(TBLSZ,3) = 0;

// first column is rounded result from original exp() calculation
itable(:,1) = round(dtable(:,1));

// column 2 is "x" of the function ( will put behing comments in the C table);
itable(:,2) = [0:TBLMAX];

// col 3 is the duty-cycle (x) as a %
itable(:,3) = round( 100 * itable(:,2) / TBLSZ );


// stringify it for the rest of the operations
stable=string(itable);

// create a column with the hex values
// x(:,1)=[0:249]
hexs = dec2hex([0:TBLMAX]') ; // transposed the row of data 
stable = cat(2,stable,hexs);

// create a column with the hex values
// x(:,1)=[0:249]
hexs = dec2hex( itable(:,1) ) ;
stable = cat(2,stable,hexs);


//Filename = fullfile(TMPDIR,"data.txt");
Filename = "dtable_s.out"; // let the output file be relative to PWD

write_csv( stable, Filename );

//
// add some info to the file (identify the data set) ... have to do this manually for now.
//
fd = mopen( Filename, 'a' );
mputl("// [1:74]  f(x) = 1500 * e ^ -x/50" , fd);
mputl("// [75:TBLSZ]  f(x) = 3x + 565" , fd);
mputl("// 10.0 * (   dtable(:,2)  +60 )  + 1700" , fd);

mclose(fd);



// gneidermeier@FWA002506 ~/gneidermeier/project/bldc_project/um0834/stm8s-discovery_dev/Project

// In Cygwin or git-bash shell, execute the following command line (relative to 
// the CWD when Scilab did write_csv() the file):
//
//  cat dtable_s.out  | sed 's/,/, /g ; s/,/, \/\/  /' > inc/model.h

// plot column 1 of the table;
plot(dtable(:,1));
//plot(itable(:,1));

