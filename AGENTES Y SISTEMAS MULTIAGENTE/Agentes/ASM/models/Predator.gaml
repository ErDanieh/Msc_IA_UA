/**
* Name: Predator
* Based on the internal empty template. 
* Author: carlos
* Tags: 
*/


model Predator

import "Animal.gaml"

global {
	list<int> huntedPrey <- [0,0,0];
}

/* Insert your model definition here */

species predator parent: animal{
	int huntSpeedMultiplier <- 5;
	
	int foodNeeded <- 5;
	
	bool hunting <- false;
	prey target_prey <- nil;
	float perception_distance <- 30.0;
	list<prey> preyTargetSpecies <- [];
	
	predicate hunt_desire <- new_predicate("hunt");
	predicate eat_desire <- new_predicate("eat");
	
	action choose_target_prey type: prey {
		if ( (target_prey = nil) or (dead (target_prey) ) ) {
			return preyTargetSpecies at_distance 30 closest_to(self);
		}
		
		return target_prey;
	}
	
	
	
	reflex check_hunger {
		if (hunger < hungerLimit and not hunting){
			target_prey <- choose_target_prey();
			if target_prey != nil{
				do remove_intention(move, false);
				do add_belief(hunt_desire);
				hunting <- true;
			}
		}
	}
	
	action updateHuntStats(list<int> statList) type: list<int> {
		int index <-0;
	 	if species(self) = cocodrile {
			index <- 0;
		} else if species(self) = leopard {
			index <- 1;
		} else if species(self) = hyena {
			index <- 2;
		}
		statList[index] <- statList[index] + 1;
		return statList;
	}
	
	perceive target: target_prey in: 1 {
		if not self.isDead {
			ask myself {
				huntedPrey <- updateHuntStats(huntedPrey);
				ask target_prey{
					aspect_image <- aspect_dead;
					isDead <- true;
					hunter <- nil;
					leader <- nil;
					do remove_intention(move, true);
					do remove_belief(escape);
					do remove_intention(escape, true);
				}	
			}
		} else {
			ask myself{
				do remove_belief(hunt_desire);
				do remove_intention(hunt_desire, true);
				do add_belief(eat_desire);
			}		
		}
	}
		
	rule belief: hunt_desire new_desire: hunt_desire strength: 5.0;
	rule belief: eat_desire new_desire: eat_desire strength: 10.0;
	
	plan hunt intention:hunt_desire {
		if dead (target_prey){
			target_prey <- nil;
			hunting <- false;
			do remove_belief(hunt_desire);
			do remove_intention(hunt_desire, true);
		} else{			
			do goto target: target_prey.location speed: 1.0 * huntSpeedMultiplier;
			do update_perception;
		}
	}
	
	plan eat intention:eat_desire {
		if not dead(target_prey){
			hunger <- hunger + 150;
			ask target_prey{
				do food_eated(myself.foodNeeded);
			}
		}
		
		target_prey <- nil;
		hunting <- false;
		do remove_belief(eat_desire);
		do remove_intention(eat_desire, true);
	}	
}

