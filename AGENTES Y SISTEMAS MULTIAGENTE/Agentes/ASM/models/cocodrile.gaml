/**
* Name: cocodrile
* Based on the internal empty template. 
* Author: daniel
* Tags: 
*/
model cocodrile

import "water.gaml"
import "antilope.gaml"
import "giraffe.gaml"
import "babuino.gaml"
import "Predator.gaml"

global {
	file cocodrile_aspect <- file("../includes/cocodrile.png");
}

species cocodrile parent: predator {
	int juniorAge <- 1;
	int matureAge <- 12;
	int oldAge <- 17;
	
	int gestationTime <- 400;
	int numberOfChilds <- 1;
	
	int timeOutsideWater <- 100;
	int thirst <- 50 min: 0 max: 50 step: 1;
	list<prey> preyTargetSpecies <- nil update: list(antilope + babuino) where(each.isDead = false or (each.isDead = true and each.cyclesDead < 50)) + list(giraffe) where(each.isDead = true and each.cyclesDead < 50);	
	
	perceive target:water in: 1{
		ask myself {
			thirst <- 50;
			timeOutsideWater <- 200;
			do remove_intention(drink, true);
			do remove_belief(drink);
		}

	}
	
	reflex addStepOutsideWater{
		timeOutsideWater <- timeOutsideWater - 1;
		
		if timeOutsideWater <  50{
			do add_belief(drink);
		}
		
		if timeOutsideWater = 0{
			do die;
		}
	}
	

	aspect base {
		draw cocodrile_aspect size: predator_size/grid_size;
	}

}