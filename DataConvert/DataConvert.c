// DataConvert.c


#include <stdio.h>
#include <stdlib.h>

//#define DEBUG_OUTPUT
#define FLT_SYNC
#define OUTLIER_CTL	20		// Maximum allowed altitude difference between adjacent samples

#define DATA_TICK   25		// TODO: Set up the data bug's output to provide this info in the data dump!

#define BIT_TRUE(x, n) ((x>>n)&1)
#define ABS(x) ((x) < 0? -(x) : (x))


int usage(char *s)
{
	printf("Usage: %s infile outfile [framesize]\n", s);
	puts("  framesize = optional number of frames for moving average altitude");
	return 1;
}


int main(int argc, char *argv[])
{
	FILE *infile;
	FILE *outfile;
	unsigned char buf[5];
	int time, frame, flight;
	double degC, alt, averageAlt;
#ifdef OUTLIER_CTL
	double altLast;
#endif
	double *altbuf = NULL;
	int altbufptr;
	short alt_int;
	unsigned char alt_dec;
	int framesize = 0;
	int i;
#ifdef FLT_SYNC
	int j;
#endif

// Check arguments
	if ((argc < 3) || (argc > 4)) return usage(argv[0]);
	if (argc == 4) {
		framesize = atoi(argv[3]);
		altbuf = (double *)malloc(framesize * sizeof(double));
	}

// Open files
	fopen_s(&infile, argv[1], "rb");
	if (!infile) {
		printf("Unable to open input file \"%s\"!\n", argv[1]);
		return 1;
	}
	if (fread(buf, 1, 5, infile) != 5) {
		printf("Unable to read input file \"%s\"!\n", argv[1]);
		return 1;
	}
	fopen_s(&outfile, argv[2], "wt");
	if (!outfile) {
		printf("Unable to create output file \"%s\"!\n", argv[2]);
		fclose(infile);
		return 1;
	}

// Write output header
	fprintf(outfile, "\"t (sec)\"\t\"Altitude (m)\"\t\"Temperature (degF)\"");
	if (framesize > 0)
		fprintf(outfile, "\t\"Smoothed Alt (m)\"\t\"FrameSize = %d\"", framesize);
	fprintf(outfile, "\n");

// Translate binary data to CSV
	time = frame = altbufptr = 0;
	flight = 1;
	while (1) {
#ifdef FLT_SYNC
		for (i=0; i<5; i++)
			if (buf[i] != 0xFF) break;
		if (i == 5) {  // means we got all 0xFF bytes
			// Read until we get a non-0xFF byte   TODO: fix this so it understands negative altitudes, though!
			while ((j = fgetc(infile)) == 0xFF)
				;
			if (j == EOF) break;
			buf[0] = j;
			if (fread(buf+1, 1, 4, infile) != 4) break;
			time = frame = altbufptr = 0;
			flight++;
		}
#endif

		// TODO: ensure negative temperatures are handled correctly!
		degC = buf[3] + BIT_TRUE(buf[4],7)*0.5 + BIT_TRUE(buf[4],6)*0.25 +
			BIT_TRUE(buf[4],5)*0.125 + BIT_TRUE(buf[4],4)*0.0625;

		alt_int = (buf[0] << 8) + buf[1];
		alt_dec = buf[2];
		if (alt_int < 0) alt_dec = ~alt_dec;
		alt = alt_int + BIT_TRUE(alt_dec,7)*0.5 + BIT_TRUE(alt_dec,6)*0.25 + BIT_TRUE(alt_dec,5)*0.125 + BIT_TRUE(alt_dec,4)*0.0625;
#ifdef OUTLIER_CTL
		if (frame > framesize) { // allow time to "settle"
			if (ABS(alt - altLast) > OUTLIER_CTL) alt = altLast;
		}
		altLast = alt;
#endif

		fprintf(outfile, "\n%d.%03d\t%.4f\t%.4f", time/1000, time%1000, alt, 32. + 9 * degC / 5);
		if (framesize > 0) {
			if (frame >= framesize) {
				averageAlt = 0;
				for (i=0; i<framesize; i++) averageAlt += altbuf[i];
				averageAlt /= (double)i;
				fprintf(outfile, "\t%.4f", averageAlt);
			}
			else {
				fprintf(outfile, "\t ");
			}
			altbuf[altbufptr] = alt;
			if (++altbufptr == framesize) altbufptr = 0;
		}

#ifdef DEBUG_OUTPUT
		fprintf(outfile, "\t");
		for (i=0; i<5; i++)
			fprintf(outfile, "%02x ", buf[i]);
//		fprintf(outfile, "::: %08x %d %d", alt, alt, alt>>8);
#endif
		if (frame == 0) fprintf(outfile, "\t\"Flt #%d\"", flight);

		time += DATA_TICK;
		frame++;
		if (fread(buf, 1, 5, infile) != 5) break;
	}

// Close up files and clean up
	fprintf(outfile, "\t\"END\"\n");
	free(altbuf);
	fclose(outfile);
	fclose(infile);

	return 0;
}

