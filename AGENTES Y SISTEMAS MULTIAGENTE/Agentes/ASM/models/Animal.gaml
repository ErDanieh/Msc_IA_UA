/**
* Name: Animal
* Based on the internal empty template. 
* Author: daniel
* Tags: 
*/
model Animal

import "Map.gaml"
import "water.gaml"
import "cocodrile.gaml"
import "hyena.gaml"

/* Insert your model definition here */
global {
	list<int> naturalDeaths <- [0,0,0,0,0,0];
	list<int> births <- [0,0,0,0,0,0];
	list<int> dehydrated <- [0,0,0,0,0,0];
	list<int> starved <- [0,0,0,0,0,0];
}

species animal skills: [moving, fipa] control: simple_bdi {
	int age;
	int stepsElapsed <- 0;
	bool isDead <- false;
	
	int speedMultiplier <- 1;
	int drinkSpeedMultiplier <- 2;
	int juniorAge <- 4;
	int matureAge <- 30;
	int oldAge <- 50;

	// 0 male 1 female
	int sex;
	bool wantReproduce <- false;
	int gestationTime <- 15;
	int gestationTimeElapsed <- 0;
	int numberOfChilds <- rnd(4);
	bool pregnant <- false;
	
	int hungerReduction <- 1;
	int hungerLimit <- 100;
	int hunger <- 200 min: 0 max: 200 step: 1;
	int thirst <- 100 min: 0 max: 100 step: 1;
	int thirstThreshold <- 50;
	int thirstDeviation <- 30;
	
	animal leader <- nil;
	water waterLocation <- nil;
	point previousLocation <- nil;
	predicate move <- new_predicate("move");
	predicate reproduce <- new_predicate("reproduce");
	predicate drink <- new_predicate("drink");
	
	rule belief: reproduce new_desire: reproduce strength: 10.0;
	rule belief: drink new_desire: drink strength: 100.0;

	//perception distance
	float perception_distance <- 10.0 parameter: true;
	geometry perceived_area <- (cone(heading - 30, heading + 30) intersection world.shape) intersection circle(perception_distance);

	init {
		previousLocation <- location;
		do add_desire(move);
	}

	reflex updateHungerAndThirst {
		if (hunger = 0 or thirst = 0) {
			int death <- rnd(5);
			if (death = 1) {
				if hunger = 0 {
					starved <- updateStats(starved);
				} else {
					dehydrated <- updateStats(dehydrated);
				}
				do die;
			}
		}

		if (thirst < thirstThreshold) {
			waterLocation <- water closest_to(self);
			do remove_intention(move, false);
			do add_belief(drink);
		}
		if(not isDead){
			hunger <- hunger - hungerReduction;
			thirst <- thirst - 1;
		}
	}

	reflex addStep {
		stepsElapsed <- stepsElapsed + 1;
		if (pregnant) {
			gestationTimeElapsed <- gestationTimeElapsed + 1;
		} else {
			gestationTimeElapsed <- 0;
		}

	}

	reflex addYear {
		if (stepsElapsed = 8760) {
			age <- age + 1;
			stepsElapsed <- 0;
		}

		if (age > juniorAge and age < oldAge) {
			wantReproduce <- true;
		}

		if (age > oldAge) {
			wantReproduce <- false;
		}
		
	}

	reflex giveBirth {
		if (pregnant and gestationTimeElapsed >= gestationTime) {

			births <- updateStats(births);
			do reproduction;
		}
	}

	reflex death {
		if (age > oldAge) {
			int death <- rnd(10);

			if (death = 1) {
				naturalDeaths <- updateStats(naturalDeaths);
				do die;
			}

		}

		//Muerte espontanea
		/*int death <- rnd(1000000);
		if (death = 1){
			do die;
		}*/
	}
	
	action updateStats(list<int> statList) type: list<int> {
		int index <-0;
		if species(self) = antilope {
			index <- 0;
		} else if species(self) = giraffe {
			index <- 1;
		} else if species(self) = babuino {
			index <- 2;
		} else if species(self) = cocodrile {
			index <- 3;
		} else if species(self) = leopard {
			index <- 4;
		} else if species(self) = hyena {
			index <- 5;
		}
		statList[index] <- statList[index] + 1;
		return statList;
	}

	action reproduction {
		create species_of(self) number: numberOfChilds {
			location <- myself.location;
			sex <- rnd(1);
			age <- 0;
			pregnant <- false;
			wantReproduce <- false;

		}
		
		pregnant <- false;
		gestationTimeElapsed <- 0;
	}

	/**Cuando esta encima del agua bebe si tiene sed*/
//	perceive target: waterLocation in: 1 when: thirst < 70{
//		ask myself {
//			waterLocation <- nil;
//			thirst <- 100;
//			do remove_intention(drink, true);
//			do remove_belief(drink);
//		}
//	}

	/**Detecta a alguien de su especie TODO: comprobar si esto funciona entre especies de predator-predator y prey-prey*/
	perceive target:species_of(self) where (each != self) in: 20 {
		if (current_plan = BDIPlan(goDrink) and waterLocation != nil) {
			do start_conversation to: [myself] protocol: 'fipa-propose' performative: 'propose' contents: [goDrink, waterLocation];
		} else if((current_plan as string) = "BDIPlan(wander)") {
			if (myself.sex != self.sex){
				if(myself.sex = 1 and myself.pregnant = false and myself.wantReproduce = true and self.wantReproduce = true){
					ask myself {
						pregnant <- true;
						wantReproduce <- false;
					}
				}
			}
			
			do start_conversation to: [myself] protocol: 'fipa-propose' performative: 'propose' contents: [wander];
		}

	}
	
	plan goDrink intention: drink {
		if waterLocation.location != location {
			previousLocation <- location;
			do goto target: waterLocation.location speed: 1.0 * drinkSpeedMultiplier;
		} else {
			thirst <- 100;
			do remove_intention(drink, true);
			do remove_belief(drink);
			heading <- heading + 180;
		}

		do update_perception;
	}

	plan wander intention: move {
		if (not isDead){
			previousLocation <- location;
			do wander amplitude: 30.0 speed: 1.0 * speedMultiplier;
			do update_perception;
		}
	}

	reflex proposals_handler when: !(empty(proposes)) {
		message proposalFromInitiator <- proposes at 0;
		
		if(proposalFromInitiator.contents[0] = "heading?") {
			do accept_proposal message: proposalFromInitiator contents: ['simon', heading];
		}else if(current_plan = BDIPlan(escape)) {
			do reject_proposal message: proposalFromInitiator contents: ['fleeing'];
		} else {
			if(proposalFromInitiator.contents[0] = goDrink) {
				if(thirst < (thirstThreshold + thirstDeviation)) {
					waterLocation <- proposalFromInitiator.contents[1];
					do remove_intention(move, false);
					do add_belief(drink);
					do accept_proposal message: proposalFromInitiator contents: ['go drink'];
				} else {
					do reject_proposal message: proposalFromInitiator contents: ['not thirsty'];
				}
			} else if(proposalFromInitiator.contents[0] = wander) {
				if((current_plan as string) = "BDIPlan(wander)" and leader = nil) {
					leader <- proposalFromInitiator.sender;
					do accept_proposal message: proposalFromInitiator contents: ['follow leader'];
				} else {
					do reject_proposal message: proposalFromInitiator contents: ['you are not my leader'];
				}
			}
		}
		
	}

	reflex speak_to_leader when: !(empty(accept_proposals)) {
		if(leader != nil) {	
			list<message> leader_proposals <- accept_proposals where (each.sender = leader);
			leader_proposals <- leader_proposals where (each.contents[0] = 'simon');
			
			if(not empty(leader_proposals)) {
				heading <- leader_proposals[0].contents[1];
			}
		}

	}
	
	reflex ask_heading_to_leader when: leader != nil {
		if ((current_plan as string) != "BDIPlan(wander)") {
			leader <- nil;
		} else {
			if (not dead(leader)){
				do start_conversation to: [leader] protocol: 'fipa-propose' performative: 'propose' contents: ["heading?"];
			}else{
				leader <- nil;
			}
		}
	}

	//computation of the perceived area
	action update_perception {
		//the agent perceived a cone (with an amplitude of 60°) at a distance of  perception_distance (the intersection with the world shape is just to limit the perception to the world)
		perceived_area <- (cone(heading - 30, heading + 30) intersection world.shape) intersection circle(perception_distance);
	}

	aspect perception {
		if (perceived_area != nil) {
			draw perceived_area color: #green;
		}
	}

	aspect leader {
		if (leader != nil) {
			draw line([location, leader.location]) color: #black;
		}
	}

}