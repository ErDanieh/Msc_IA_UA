/**
* Name: leopard
* Based on the internal empty template. 
* Author: carlos
* Tags: 
*/
model leopard

import "Predator.gaml"
import "antilope.gaml"

global {
	file leopard_aspect <- file("../includes/leopard.png");
}

species leopard parent: predator {
	int juniorAge <- 3;
	int matureAge <- 12;
	int oldAge <- 17;
	
	int gestationTime <- 400;
	int numberOfChilds <- 1; 
	int hungerReduction <- 1;
	int hungerLimit <- 128;
	int foodNeeded <- 4;
	
	list<prey> preyTargetSpecies <- nil update: list(antilope + babuino) where(each.isDead = false or (each.isDead = true and each.cyclesDead < 50)) + list(giraffe) where(each.isDead = true and each.cyclesDead < 50);

	
	aspect base {
		float s <- predator_size/grid_size;
		draw leopard_aspect size: s;
	}
}