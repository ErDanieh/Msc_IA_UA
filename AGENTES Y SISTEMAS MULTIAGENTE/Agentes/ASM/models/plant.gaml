/**
* Name: Arbol
* Based on the internal empty template. 
* Author: arp12
* Tags: 
*/


model plant

global {
	file grass <- file("../includes/wheat_yellow.png");
}

species plant {
    float max_food <- 20.0 ;
    float food_prod <- rnd(1.0) ;
    float food <- rnd(20.0) max: max_food update: food + food_prod;
    rgb color <- rgb(int(255 * (20 - food)), 255, int(255 * (20 - food))) 
         update: rgb(int(255 * (20 - food)), 255, int(255 * (20 - food))) ;
    action eat{
    	food<-food -1.0;
    }
	aspect base {
		float s <- plant_size / grid_size;
		draw grass size: s;
	}
}