//
// curve_gen;
//  generate a exponential decay function with paramters;
//  dump data to csv;
// 11/2020; G. Neidermeier;
//


// clear the Scilab workspace;

clear;


// args for Tau, A, and x/y offsets ????
TBLSZ=90 // 250
TBLMAX = TBLSZ - 1



A=51000 // 
TAU=0.011  // can'y go too small, the low speeds numbers are too high but is more straight
Y_INT=-400 // negative y offset to make start numbers lower
X_INT=310 // bigger x offset helps make start numbers lower


// starts in time or slightly advanced and beomes recesses around $30 (48)
A=72000 // 
TAU=0.012  // can'y go too small, the low speeds numbers are too high but is more straight
Y_INT=-400 // negative y offset to make start numbers lower
X_INT=310 // bigger x offset helps make start numbers lower


// starts in time or slightly advanced and beomes recesses around $30 (48)
A=72000 // 
TAU=0.012  // can'y go too small, the low speeds numbers are too high but is more straight
Y_INT=-400 // negative y offset to make start numbers lower
X_INT=310 // bigger x offset helps make start numbers lower





//new table with 2 fields and 250 rows;
dtable(TBLSZ,2)=0;

// column 2 is "x" of the function ( will put behing comments in the C table);
dtable(:,2)=[0:TBLMAX];

// column 1 is the table value;
dtable(:,1) =  Y_INT +  A * exp( -TAU *  ( X_INT + dtable(:,2) )   );


//plot(   1 -  20 * dtable(:,2)  + 1400)
dtable(:,1) = 1 -  14 * (   dtable(:,2)  +30 )  + 1750  // never advances
dtable(:,1) = 1 -  14.00 * (   dtable(:,2)  +30 )  + 1780  //  eventually advances ($47)
dtable(:,1) = 1 -  10.5 * (   dtable(:,2)  +55 )  + 1700  // dies at $50
dtable(:,1) = 1 -  10.0 * (   dtable(:,2)  +60 )  + 1700  // dies at $52
//dtable(:,1) = 1 -  9.8 * (   dtable(:,2)  +62 )  + 1700  // nfg



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
mputl("// 10.0 * (   dtable(:,2)  +60 )  + 1700" , fd);
mclose(fd);



// gneidermeier@FWA002506 ~/gneidermeier/project/bldc_project/um0834/stm8s-discovery_dev/Project

// In Cygwin or git-bash shell, execute the following command line (relative to 
// the CWD when Scilab did write_csv() the file):
//
//  cat ../../../DTABLE.OUT | sed 's/\r// ; s/,/,  \/\/ /' > stm8_mcp/inc/dtable.h

// plot column 1 of the table;
plot(dtable(:,1));
//plot(itable(:,1));

