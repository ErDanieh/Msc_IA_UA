/**
* Name: Map
* Based on the internal empty template. 
* Author: daniel
* Tags: 
*/


model Map

import "hyena.gaml"
import "cocodrile.gaml"
import "antilope.gaml"
import "leopard.gaml"
import "babuino.gaml"
import "giraffe.gaml"

import "plant.gaml"
import "water.gaml"
import "tree.gaml"

global {
	matrix cell_fields<- generate_terrain(4, grid_size, grid_size, 1, 0.5, 1) as_matrix {50,50};
}


grid grille width: grid_size height: grid_size {
	rgb color <- rgb(229, 209, 166);
	
	point displace(point value) {
		float rand_x <- rnd(-1.0, 1.0);
		float rand_y <- rnd(-1.0, 1.0);
		return {value.x + rand_x, value.y + rand_y};
	}
	
	init {
		point original_loc <- (grille grid_at {grid_x, grid_y}).location;
		point loc <- displace(original_loc);
		
		// Tree generation
		float minimum <- min(cell_fields) as float;
		float threshold <- cell_fields[grid_x, grid_y] + abs(minimum);
		if(threshold < 0.12) { // Water
			create water with:[location: original_loc, color: #blue];
		} else if(between(threshold, 0.15, 0.2)) { // Fruit tree
			create tree with:[location: loc, type: "fruit"];
		} else if(between(threshold, 0.2, 0.28)){ // Tree and monkey		
			if (init_nb_babuino != length(list(babuino))) {
				create babuino with: [
					location: loc,
					age: rnd(0,5),
					sex: rnd(1),
					wantReproduce: false
				];
			}
		
			create tree with:[location: loc, type: "leaf"];
		} else if(between(threshold, 0.28, 0.4) and flip(0.75)) { // Plants
			create plant with:[location: loc];
		} else if(between(threshold, 0.7, 0.8)) { // Dead tree
			create tree with:[location: loc, type: "none"];
		}
		
		if(between(threshold, 0.35, 0.55) and flip(0.4) and init_nb_antilope != length(list(antilope))) {
			create antilope with: [location: loc, age: rnd(0,5), sex: flip(0.33333333), wantReproduce: false, hunger: rnd(50,200), thirst: rnd(50,100)];
		}
		
		if(between(threshold, 0.35, 0.55) and flip(0.08) and init_nb_giraffe != length(list(giraffe))) {
			create giraffe with: [location: loc, age: rnd(0,5), sex: rnd(1), wantReproduce: false, hunger: rnd(50,200), thirst: rnd(50,100)];
		}
		
		if(threshold > 0.12 and flip(0.08) and init_nb_leopard != length(list(leopard))) {
			create leopard with: [location: loc, age: rnd(0,5), sex: flip(0.33333333), wantReproduce: false, hunger: rnd(50,200), thirst: rnd(50,100)];
		}
		
		if(threshold < 0.15 and flip(0.15) and init_nb_cocodrile != length(list(cocodrile))) {
			create cocodrile with: [location: loc, age: rnd(0, 5), sex: flip(0.5), wantReproduce: true, hunger: rnd(50,200), thirst: rnd(50,100)];
		}
		
		if(threshold > 0.12 and flip(0.08) and init_nb_hyena != length(list(hyena))) {
			create hyena with: [location: loc, age: rnd(0, 5), sex: flip(0.5), wantReproduce: true, hunger: rnd(50,200), thirst: rnd(50,100)];
		}
		
		/*if(between(threshold, 0.35, 0.55) and flip(0.4) and init_nb_hyena != length(list(hyena))) {
			create hyena with: [location: loc, age: rnd(0,5), sex: flip(0.33333333), wantReproduce: false];
		}*/
	}
}