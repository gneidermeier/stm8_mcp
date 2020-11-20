//
// curve_gen;
//  generate a exponential decay function with paramters;
//  dump data to csv;
// 11/2020; G. Neidermeier;
//


// clear the Scilab workspace;

clear;


// how to do args for Tau, A, and x/y offsets ????
TBLSZ=250
TBLMAX = TBLSZ - 1

A=920
TAU=0.02
Y_INT=0
X_INT=0

A=4000
TAU=0.04  // 1/25
Y_INT=150
X_INT=40

A=4000
TAU=0.04  // 1/25
Y_INT=200
X_INT=50



// dies at 58d, (timing is recessed)
A=5200
TAU=0.02  // 1/25
Y_INT=200
X_INT=50


A=5200
TAU=0.025  // 1/25
Y_INT=200
X_INT=50

A=5200
TAU=0.025  // 1/25
Y_INT=200
X_INT=50



//new table with 2 fields and 250 rows;
dtable(TBLSZ,2)=0;

// column 2 is "x" of the function ( will put behing comments in the C table);
dtable(:,2)=[0:TBLMAX];

// column 1 is the table value;
dtable(:,1) =  Y_INT +  A * exp( -TAU *  ( X_INT + dtable(:,2) )   );


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
hexs = dec2hex([0:249]') ; // transposed the row of data 
stable = cat(2,stable,hexs);

// create a column with the hex values
// x(:,1)=[0:249]
hexs = dec2hex( itable(:,1) ) ;
stable = cat(2,stable,hexs);

write_csv(stable, "dtable_s.out");




//save to csv;
write_csv(itable, "dtable.out");


// gneidermeier@FWA002506 ~/gneidermeier/project/bldc_project/um0834/stm8s-discovery_dev/Project

// In Cygwin or git-bash shell, execute the following command line (relative to 
// the CWD when Scilab did write_csv() the file):
//
//  cat ../../../DTABLE.OUT | sed 's/\r// ; s/,/,  \/\/ /' > stm8_mcp/inc/dtable.h

// plot column 1 of the table;
plot(dtable(:,1));
plot(itable(:,1));

