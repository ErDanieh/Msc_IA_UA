/**
* Name: Prey
* Based on the internal empty template. 
* Author: pdrod
* Tags: 
*/
model Prey

import "Predator.gaml"
import "Animal.gaml"
import "plant.gaml"

global {
}

/* Insert your model definition here */
species prey parent: animal {
	image aspect_image <- nil;
	image aspect_dead <- nil;
	agent target_plant <- nil;
	predator hunter <- nil;
	float speed <- -3.0;
	float speedMultiplier <- (200 - hunger) / (hunger);
	list<agent> plantSpecies <- [];
	predicate escape <- new_predicate("escape");
	predicate eating <- new_predicate("eating");
	
	int weight <- 10;
	int cyclesDead <- 0 update: cyclesDead + (1 * (isDead as int));
	int decompositionTime <- 10;
	
	action food_eated(int food){
		weight <- weight - food;
	}
	
	reflex destroy{
		if (weight <= 0 or cyclesDead >= decompositionTime){
			do die;
		}
	}
	
	perceive target:agents of_generic_species predator where(each.hunting) in: 8 when: not isDead{ 
		focus id:"predator" var:location strength:10.0; 
		
		ask myself {
			do remove_intention(move, false);
			do add_belief(escape);
			hunter <- who_i_need_to_escape();
		}
	}
	action who_i_need_to_escape type: predator {
		if (hunter = nil) {
			return (agents of_generic_species predator at_distance 8 where (each.hunting)) closest_to (self);
		}

		return hunter;
	}

	action choose_target_plant type: agent {
		if (target_plant = nil) {
			return plantSpecies at_distance 30 closest_to (self);
		}

		return target_plant;
	}

	reflex check_hunger {
		if (hunger < 100) {
			target_plant <- choose_target_plant();
			if target_plant != nil {
				do remove_intention(move, false);
				do add_belief(eating);
			}

		}

	}

	rule belief: eating new_desire: eating strength: 20 * ((200 - hunger) / 200);
	rule belief: escape new_desire: escape strength: 10.0;
	
	plan escaping intention: escape {
		if dead (hunter){
			hunter <- nil;
			do remove_belief(escape);
			do remove_intention(escape, true);
		}else if (self distance_to hunter > 20) {
			hunter <- nil;
			do remove_belief(escape);
			do remove_intention(escape, true);
		} else {
			previousLocation <- location;
			do goto target: hunter.location speed: speed * speedMultiplier;
			if location.x <= 0 or location.x >= 99 or location.y <= 0 or location.y >= 99 {
				location <- previousLocation;
			}

		}
	}

	plan eat_grass intention: eating {
		if hunger >= 180 {
			target_plant <- nil;
			do remove_belief(eating);
			do remove_intention(eating, true);
		} else {
			if (self distance_to target_plant < 1) {
				float it_is_empty <- 0.0;
				ask target_plant as: plant {
					it_is_empty <- food;
				}

				if it_is_empty > 1.0 {
					hunger <- hunger + 40;
					ask target_plant as: plant {
						do eat();
					}

				} else {
					target_plant <- choose_target_plant();
				}

			} else {
				do goto target: target_plant.location speed: 1.0 * speedMultiplier;
			}

		}
		
		do update_perception;
	}

	aspect base {
		float s <- prey_size/grid_size;
		draw aspect_image size: s;
	}

}

