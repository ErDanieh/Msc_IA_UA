/**
* Name: babuino
* Based on the internal empty template. 
* Author: pdrod
* Tags: 
*/


model babuino
import "Animal.gaml"
import "plant.gaml"
import "leopard.gaml"
import "Prey.gaml"

global {
	image aspect_alive_babuino <- image("../includes/babuino.png");
	image aspect_dead_babuino <-image("../includes/dead_babuino.png");
}


species babuino parent: prey{
	image aspect_image <- aspect_alive_babuino;
	image aspect_dead <- aspect_dead_babuino;
	int speedMultiplier <- 2;
	int gestationTime <- 800;
	int juniorAge <- 2;
	int matureAge <- 18;
	int oldAge <- 27;
	tree actualTree<- nil;
	
	int weight <- 20;
	int cyclesDead <- 0 update: cyclesDead + (1 * (isDead as int));
	int decompositionTime <- 168;
	
	list<agent> plantSpecies <- []update: list(tree);
	
	action choose_home type:tree{
		if (actualTree = nil) {
			tree home <- (agents of_generic_species tree at_distance 30 where((each.type="fruit") and each.food>0)) closest_to(self);
			if home!=nil{
				return home;
			}
		}
		
		return actualTree;
	}
	
	reflex check_tree {
		if (actualTree=nil){
			actualTree <- choose_home();
		}
	}
	
	reflex asking_home {
		babuino monkey<-agents of_generic_species babuino at_distance 5 closest_to(self);
		if (actualTree=nil and monkey!=nil){
			tree home <-nil;
			ask monkey as:babuino{
				if actualTree!=nil{
					home<-actualTree;
				}
			}
			actualTree<-home;
		}
	}
	
	plan eat_grass intention:eating {
		if hunger>=120{
							
			do remove_belief(eating);
			do remove_intention(eating, true);
		}else{
			if	actualTree=nil{
				actualTree <- choose_home();
				do wander amplitude: 30 speed: 1.0 * speedMultiplier;
			}else{
				if (self distance_to actualTree<1){
					float It_is_empty<-0.0;
					ask actualTree as: tree {
						It_is_empty<-food;
					}
					if It_is_empty>1.0{
						hunger<-hunger+40;
						ask actualTree as: tree {
							do eat();
						}
					}else{
						actualTree <- choose_home();
					}
				}else{
					do goto target: actualTree.location speed: 1.0 * speedMultiplier;
				}
			}		
		}
	}
	
	plan wander intention: move {
		previousLocation <- location;
		if actualTree!=nil{
			if(self distance_to actualTree>15){
				actualTree <- choose_home();
			}else if self distance_to actualTree<16{
				do wander amplitude: rnd(0,30) speed: rnd(-1.0,1.0) * speedMultiplier;
			}else{
				do goto target:actualTree speed: 1.0 * speedMultiplier; 
			}
			do update_perception;
		}else{
			do wander amplitude: 30 speed: 1.0 * speedMultiplier;
		}
	}
	
	aspect base {
	  draw aspect_image size: 1;
	}
	
}
