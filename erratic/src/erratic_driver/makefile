# Author: Joakim Arfvidsson

CPP_FLAGS = -std=c++11

DISTFILES = README.txt main.cpp robot_packets.cpp robot_packets.h robot_params.h makefile
DISTNAME = robot_client-`date +"%Y-%m-%d"`

default: robot_client

robot_client: main.cpp robot_packets.cpp robot_packets.h robot_params.h makefile
	g++ ${CPP_FLAGS} main.cpp robot_packets.cpp -o robot_client

install: robot_client
	cp robot_client /usr/local/bin/

dist: $(DISTFILES)
	mkdir -p dists
	rm -rf "dists/$(DISTNAME)"
	mkdir "dists/$(DISTNAME)"
	cp $(DISTFILES) "dists/$(DISTNAME)"
	cd dists; zip -r "$(DISTNAME)" "$(DISTNAME)"
	rm -rf "dists/$(DISTNAME)"

clean:
	rm -rf *.o robot_client *.out