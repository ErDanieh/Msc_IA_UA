/**
* Name: leopard
* Based on the internal empty template. 
* Author: carlos
* Tags: 
*/

model hyena

import "Predator.gaml"
import "antilope.gaml"


global {
	image hyena_aspect <- image("../includes/hyena.png");
}

species hyena parent: predator{
	
	int juniorAge <- 3;
	int matureAge <- 12;
	int oldAge <- 17;
	
	int gestationTime <- 400; 
	int numberOfChilds <- rnd(1,2); 
	
	int hungerLimit <- 115;
	int foodNeeded <- 6;
	
	list<prey> preyTargetSpecies <- nil update: list(antilope + giraffe + babuino) where(each.isDead = true);
	
	aspect base {
	  draw hyena_aspect size: 3;
	}
}