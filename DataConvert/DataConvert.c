// DataConvert.c


#include <stdio.h>
#include <stdlib.h>


#define DATA_TICK   25

#define BIT_TRUE(x, n) ((x>>n)&1)


int main(int argc, char *argv[])
{
	FILE *infile;
	FILE *outfile;
	unsigned char buf[5];
	int time = 0;
	double degC;
	short alt;
	unsigned char alt_dec;
	int i;

// Open files
	fopen_s(&infile, argv[1], "rb");
	if (!infile) {
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
	fputs("\"t (sec)\"\t\"Altitude (m)\"\t\"Temperature (degF)\"", outfile);

// Translate binary data to CSV
	while (fread(buf, 1, 5, infile) == 5) {
		// may not do negative temperatures correctly!
		degC = buf[3] + BIT_TRUE(buf[4],7)*0.5 + BIT_TRUE(buf[4],6)*0.25 +
			BIT_TRUE(buf[4],5)*0.125 + BIT_TRUE(buf[4],4)*0.0625;
		alt = (buf[0] << 8) + buf[1];
		alt_dec = buf[2];
		if (alt < 0) alt_dec = ~alt_dec;
		fprintf(outfile, "\n%d.%03d\t%d.%04d\t%.4f\t", time/1000, time%1000, alt,
			BIT_TRUE(alt_dec,7)*5000 + BIT_TRUE(alt_dec,6)*2500 + BIT_TRUE(alt_dec,5)*1250 + BIT_TRUE(alt_dec,4)*625,
			32. + 9 * degC / 5);
		for (i=0; i<5; i++)
			fprintf(outfile, "%02x ", buf[i]);
//		fprintf(outfile, "::: %08x %d %d", alt, alt, alt>>8);
		time += DATA_TICK;
	}

// Close up files
	fclose(outfile);
	fclose(infile);

	return 0;
}

