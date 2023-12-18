/**
* Name: Plantilla P1 Wumpus. ASM 23/24
* Author: Fidel
* Tags: 
*/

model Wumpus_template


global {
	
	
	int displatTextSize <-2;
	
	predicate move_random_desire <- new_predicate("move_random");
	predicate want_escape <- new_predicate("want_escape", true);
	predicate getGold <- new_predicate("getGold", true);
	
	int  trampasEvitadas <- 0;
	int casillasExplorada <- 0;
	
	
	string glitterLocation <- "glitterLocation";

	init {
		create goldArea number:1;
		create wumpusArea number: 1;
		create pitArea number: 9;
		create player number:1;
	}
	
}

/**Agente que representa al mundo */
grid gworld width: 25 height: 25 neighbors:4 {
	rgb color <- #green;
}

/**Agente del jugador */
species player skills: [moving] control: simple_bdi {
	
	
	// Posicion de inicio del jugador
	gworld maCellule <- one_of(gworld);
	// Posicion anterior del jugador
	gworld pastCellule <- maCellule;

	
	init {
		location<-maCellule.location;
		do add_desire(move_random_desire);
		
	}
	
	
	
	/**Detecta una zona de brillo y para al agente encima */
	perceive target:glitterArea in: 1{ 
		
		focus id:"glitterLocation" var:location strength:10.0; 
		
		ask myself{
			do remove_intention(want_escape, true);
			do remove_intention(move_random_desire, true);
		}
		
	}
	
	
	//Detecta el tesoro
	perceive target:goldArea in: 1{
		
		ask myself{
			do remove_belief( get_predicate(get_belief_with_name("glitterLocation")));
			do remove_intention(getGold,true);
			do remove_intention(want_escape, true);
			do remove_intention(move_random_desire, true);
			ask world{
				do pause;
			}
				
		}
		
	}
	
	/**Detecta una zona de brisa y hace retroceder al agente una casillas */
	perceive target:breezeArea in: 1{ 
		
		ask myself{
			do remove_intention(move_random_desire, true);
			do add_desire(want_escape);
		}
	}
	
	/**Detecta una zona de edor y hace retroceder al agente una casillas */
	perceive target:odorArea in: 1{ 
		
		ask myself{
			do remove_intention(move_random_desire, true);
			do add_desire(want_escape);
		}
	}
	
	//Creencia de busqueda de oro
	rule belief: new_predicate(glitterLocation) new_desire: getGold;
	
	
	//Exploracion celulas cercanas al oro
	plan exploreAroundGlitter intention: getGold {
		
		map glitter <- get_predicate(get_belief_with_name("glitterLocation")).values("location_value");
		gworld celdaGlitter <- gworld(glitter['location_value']);
		gworld celdaActual <- gworld({location.x, location.y});
		
		
		if (celdaGlitter = celdaActual){
			list<gworld> vecinos <- [];
			ask celdaGlitter{
				vecinos <- neighbors;
			}
			
			gworld vecino <- one_of(vecinos);
			casillasExplorada <- casillasExplorada + 1;
			do goto target: vecino on: gworld speed: 24.0;	
		}
		else{
			casillasExplorada <- casillasExplorada + 1;
			do goto target: celdaGlitter on: gworld speed: 24.0;	
		}
		
        
	}
	
	
	
	/**Hace escapar al agente retrocediendo una casilla */
	plan escape intention:want_escape {
		do goto target: pastCellule on: gworld speed: 24.0;
		trampasEvitadas <- trampasEvitadas + 1;
		casillasExplorada <- casillasExplorada + 1;
		do remove_intention(want_escape, false);
		do add_desire(move_random_desire);
	}
	
	
	
	/**Plan de patrulla para moverse por el mapa libremente */
	plan patrolling intention:move_random_desire {
		
		gworld playerPosition <-gworld({location.x, location.y});
		pastCellule <- playerPosition;
        
        list<gworld> vecinos <- [];
        ask playerPosition {
            vecinos <- neighbors;
        }
        
        /**Me guardo mi posicion anterior */
        gworld vecino <- one_of(vecinos);
        casillasExplorada <- casillasExplorada + 1;
        do goto target: vecino on: gworld speed: 24.0;
	}
	
	
	aspect base{
		draw circle(2) color: #purple;
		//Informacion actual del jugador
		draw ("curIntention:" + get_current_intention()) color:#black size:displatTextSize at:{location.x-2,location.y+2*displatTextSize};
		draw ("TrampasEvitadas: " + trampasEvitadas ) color:#black size:displatTextSize at:{location.x-2,location.y+4*displatTextSize};
		draw ("CasillasExploradas: " + casillasExplorada) color:#black size:displatTextSize at:{location.x-2,location.y+6*displatTextSize};
	}
	

}


/**Esto es un agente */
species odorArea{
	aspect base {
	  draw square(4) color: #brown border: #black;		
	}
}


/**Esto es otro agente */
species wumpusArea{
	init {
		gworld place <- one_of(gworld);
		location <- place.location;
		
		//Place es un cell, y por tanto puedo solicitar sus vecinos https://gama-platform.org/wiki/GridSpecies
		list<gworld> vecinos <- [];
		ask place {
			vecinos <- neighbors;
		}
		
		loop i over: vecinos {
			create odorArea{
				location <- i.location;
			}
		}
	}
	aspect base {
	  draw square(4) color: #red border: #black;
	}
}

/**Esto es otro agente */
species glitterArea{
	aspect base {
	  draw square(4) color: #chartreuse border: #black;
	}
}

/**Esto es otro agente */
species goldArea{
	gworld place <- one_of(gworld);
	
	init {
		
		location <- place.location;
		
		//Place es un cell, y por tanto puedo solicitar sus vecinos https://gama-platform.org/wiki/GridSpecies
		list<gworld> vecinos <- [];
		ask place {
			vecinos <- neighbors;
		}
		
		loop i over: vecinos {
			create glitterArea{
				location <- i.location;
			}
		}
	
	}
	
	aspect base {
	  draw square(4) color: #yellow border: #black;		
	}
}



// Area de brisa para el pozo
species breezeArea{
	aspect base {
	  draw square(4) color: #blue border: #black;		
	}
}

// Specie del pozo
species pitArea{
	init {
		gworld place <- one_of(gworld);
		location <- place.location;
		
		//Place es un cell, y por tanto puedo solicitar sus vecinos https://gama-platform.org/wiki/GridSpecies
		list<gworld> vecinos <- [];
		ask place {
			vecinos <- neighbors;
		}
		
		loop i over: vecinos {
			create breezeArea{
				location <- i.location;
			}
		}
	
	}
	
	aspect base {
	  draw square(4) color: #darkblue border: #black;		
	}
}


/**Define la manera en la que se va a ejecutar */
experiment Wumpus_experimento_1 type: gui {
	/** Insert here the definition of the input and output of the model */

	
	output {					
		
		/**Pantalla del experimento */
		display view1 { 
			grid gworld border: #darkgreen;
			
			species goldArea aspect:base;
			species glitterArea aspect:base;
			
			species pitArea aspect:base;
			species breezeArea aspect:base;
			
			species wumpusArea aspect:base;
			species odorArea aspect:base;
			
			species player aspect:base;

		}
		
		
		display chart{
				chart "Grafica" type: series {
				data "casillasExploradas" value: casillasExplorada color: #red;
				data "trampasEvitadas" value: trampasEvitadas color: #blue;
			}
		}
		
	}
}
