/**
* Name: antilope
* Based on the internal empty template. 
* Author: pedro
* Tags: 
*/
model antilope

import "plant.gaml"
import "Prey.gaml"

global {	
	image aspect_alive_antiloope <- image("../includes/antilope.png");
	image aspect_dead_antiloope <-image("../includes/dead_antilope.png");
}


species antilope parent: prey{
	image aspect_image <- aspect_alive_antiloope;
	image aspect_dead <- aspect_dead_antiloope;
	int speedMultiplier <- 1;
	
	int juniorAge <- 2;
	int matureAge <- 5;
	int oldAge <- 9;
	
	int gestationTime <- 800;	
	int numberOfChilds <- 1; 
	
	int weight <- 40;
	int cyclesDead <- 0 update: cyclesDead + (1 * (isDead as int));
	int decompositionTime <- 168;
	
	list<plant> plantSpecies <- [] update: list(plant);
}
