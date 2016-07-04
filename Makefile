CC=g++ 
CFLAGS=-O3 -w -c -std=c++11 -fopenmp -Wall
#CFLAGS=-O0 -w -c -g -std=c++11 -fopenmp -Wall

THEIAPATH=/home/rajvi/PhD/Projects/ThirdPartyCodes/TheiaLatestTranslation/TheiaSfM/

LIBPATH=-L$(THEIAPATH)/theia-build/lib/ -L/usr/local/lib/ -L/usr/lib/ -L/home/rajvi/Bundler/lib/ann_1.1_char/lib/ -L/home/rajvi/PhD/Projects/ThirdPartyCodes/GlobalSFM/ceres-solver/build/lib/
IFLAGS=-I/usr/local/include/ -I/usr/include/ -I/home/rajvi/PhD/Projects/openMVG-master/src/third_party/eigen/ -I/home/rajvi/Bundler/lib/ann_1.1_char/include/ -I$(THEIAPATH)/libraries/ -I$(THEIAPATH)/TheiaSfM/libraries/ -I/usr/local/include/theia/libraries/vlfeat/ -I/usr/local/include/theia/libraries/statx -I/usr/local/include/theia/libraries/optimo/  

LIBS=-lANN_char -lz -lglog -lceres -lstlplus3 -leasyexif -ltheia -leasyexif -lceres 

default: mg2vg 
	mv -t ../bin/ mg2vg

clean: 
	rm *.o mg2vg
	
mg2vg: MatchGraph2ViewGraph.o Reader.o keys2a.o
	$(CC) $(IFLAGS) MatchGraph2ViewGraph.o Reader.o keys2a.o $(LIBPATH) -Wall -o mg2vg $(LIBS)

MatchGraph2ViewGraph.o: MatchGraph2ViewGraph.cpp
	$(CC) $(CFLAGS) $(IFLAGS) MatchGraph2ViewGraph.cpp 

Reader.o: Reader.cpp Reader.h
	$(CC) $(CFLAGS) $(IFLAGS) Reader.cpp

keys2a.o: keys2a.cpp keys2a.h
	$(CC) $(CFLAGS) $(IFLAGS) keys2a.cpp
