/**
* Name: Arbol
* Based on the internal empty template. 
* Author: arp12
* Tags: 
*/


model tree

global {
	file dead <- file("../includes/dead.svg");
	file acacia <- file("../includes/acacia.svg");
	file baobab <- file("../includes/baobab.svg");
}

species tree {
	float max_food <- 0.0;
    float food_prod <- 0.0;
    float food <- 0.0 max: max_food update: food + food_prod;
	string type;
	
	init {
		if(type = "fruit") {
			max_food <- 50.0;
			food_prod <- rnd(0.1);
			food <- rnd(50.0);
		} else if(type = "leaf") {
			max_food <- 50.0;
			food_prod <- rnd(0.1);
			food <- rnd(50.0);
		}
	}
	action eat{
    	food<-food -0.5;
    }
	aspect base {
		float s <- tree_size/grid_size;
		if(type = "none") {
			draw dead size: s color: rgb(182, 140, 99);
		} else if(type = "leaf") {
			draw acacia size: s color: rgb(104, 170, 74);
		} else {
			draw baobab size: s color: #orange;	
		}
	}
}