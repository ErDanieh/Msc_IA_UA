/**
* Name: watter
* Based on the internal empty template. 
* Author: arp12
* Tags: 
*/
model water

species water {
	rgb color;

	init {
	}

	aspect base {
		float s <- water_size / grid_size;
		draw circle(s) color: color;
	}

}