/**
* Name: giraffe
* Based on the internal empty template. 
* Author: pdrod
* Tags: 
*/


model giraffe

import "Animal.gaml"
import "plant.gaml"
import "leopard.gaml"
import "Prey.gaml"


global {
	image aspect_alive_jirafa <- image("../includes/jirafa.png");
	image aspect_dead_jirafa <-image("../includes/dead_jirafa.png");
}

species giraffe parent: prey{
	image aspect_image <- aspect_alive_jirafa;
	image aspect_dead <- aspect_dead_jirafa;
	int speedMultiplier <- 2;
	
	int gestationTime <- 1600;
	int juniorAge <- 5;
	int matureAge <- 10;
	int oldAge <- 25;
	
	int weight <- 800;
	int cyclesDead <- 0 update: cyclesDead + (1 * (isDead as int));
	int decompositionTime <- 168;
	
	list<agent> plantSpecies <- []update: list(tree);
	
	
	plan eat_grass intention:eating {
		if hunger>=120{
			target_plant <- nil;
							
			do remove_belief(eating);
			do remove_intention(eating, true);
		} else {			
			if (self distance_to target_plant<1){
				float It_is_empty<-0.0;
				ask target_plant as:tree {
					It_is_empty<-food;
				}
				if It_is_empty>1.0{
					hunger<-hunger+40;
					ask target_plant as:tree {
						do eat();
					}
				} else {
					target_plant <- choose_target_plant();
				}
			} else {
				do goto target: target_plant.location speed: 1.0 * speedMultiplier;
			}
		}
	}
	
	aspect base {
	  draw aspect_image size: 6;
	}
	
}

