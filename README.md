# Artificial Life
UPDATED FOR FINAL

#PROJECT
Contains my work for CS 396 Artifical Life. Purpose is to develop an evolutionary algorithm that optimizes a creature to meet a certain fitness criteria in physics rendering simulator mujoco. Folders correspond to homework assignments--each folder contains a xml file (output) and a py file (simulation code).


#HOW TO RUN
Run the .py file and it'll write to the xml file, which will then be used to build the simulation. You can observe the parameters (eg. parents, children) used to generate the species and adjust to your liking. 


#FUNCTIONS
code can be split into different sections: graph function for initiating creature genotype, helper functions to ensure properly positioned elements, initialize functions for assembling the creature, builder functions for translating info to xml code, mutation and fitness functions, and the simulation functions. most of the core functionality lies in the init_creature_geno(), generate_pheno_traits(), build_creature_pheno(), and mutate_geno/pheno() functions. 


#FITNESS
fitness is measured by distance traveled for this evolutionary algorithm. this is calculated with the math module's .dist() function, with mujoco's .xpos being passed in as start/end points. 


#MUTATIONS
mutations are generated based on geno and pheno of the parent generation. available traits to be mutated include: body size/shape, leg size/shape, # of legs, # of leg segments. with each mutation, only 1 trait from geno is mutated, whether it's # of legs or # of segments, and only 1 trait from pheno is mutated whether its leg/body size/shape. 

#EVOLUTION
in my demo video, I test two approaches to evolution -- large population (increased diversity) with fewer generations (decreased selection), and small population (decreased diversity) with more generations (increased selections) to observe which would output a more optimal creature. I ran each method 3 times, with 30/10 population/generations and 10/30 population/generations. 

#RESULTS 
30/10 - trial 1
      Initial       |        Final
------------------- | -------------------
1.7973785339956396  | 20.854240184116758
0.9171544102522634  | 17.77676015343553
0.8359594181136887  | 4.437672601455963
0.7701665962844884  | 3.2578985279487753
0.7478775738040783  | 3.067742135412923
0.6485630367247291  | 2.426021472568923
0.6374313845834549  | 2.071040991968456
0.5860361988929675  | 2.056063249000716
0.5330979096329553  | 1.9709201474363296
0.4547061317766159  | 1.9544491131936528
0.4236502000049396  | 1.9189996808141996
0.4198292755146421  | 1.8804043167427589
0.4066598885756013  | 1.8277126057793611
0.3902910033424688  | 1.7973785339956396
0.3463278321058344  | 1.7710209753371264

30/10 - trial 2
      Initial       |        Final
------------------- | -------------------
2.5084540423981574  | 2.5084540423981574
1.8274920289526453  | 2.074607014158414
1.7451052096548607  | 2.0091525329015916
1.1704851588119172  | 1.9982530065355197
1.0911057633229961  | 1.8838240795109369
0.7988789885537816  | 1.8274920289526453
0.7262026367137726  | 1.7763427539855692
0.6071787110272981  | 1.7695836323279468
0.5389313206032250  | 1.7451052096548607
0.5013891488845912  | 1.712775717854158
0.4996739242849438  | 1.6907378063150227
0.4648635640185634  | 1.6264539885747151
0.4385398164437661  | 1.6197999117384587
0.4312970039477021  | 1.5820833651899064
0.4218703729140981  | 1.5802737336425683

30/10 - trial 3
      Initial       |        Final
------------------- | -------------------
1.7571477919378546  | 2.1630713741320196
1.1454115328813586  | 2.029709353408485
0.8665221994756224  | 2.024936400182996
0.8418349944429167  | 2.0021944309192037
0.8167531068145996  | 1.9507955786784108
0.7641030820927139  | 1.925461548457292
0.5706510519790434  | 1.9064939202636084
0.5383163417939898  | 1.8494888068662299
0.5005054387280405  | 1.842305010994384
0.4911830779467437  | 1.8381970373015017
0.4361717205645398  | 1.830913482526689
0.4231801421447130  | 1.8243975341956251
0.3965729849743455  | 1.8086491217778893
0.3602569932377558  | 1.7952330194689974
0.3374073071870890  | 1.7571477919378546


10/30 - trial 1
      Initial       |        Final
------------------- | -------------------
1.3907221394203200  | 1.8936913784993834
1.1505980668965880  | 1.8035604914961723
0.7085152771633342  | 1.7266218181210702
0.5882393548031277  | 1.7253502074308813
0.3830490905030076  | 1.722720094227968

10/30 - trial 2
      Initial       |        Final
------------------- | -------------------
1.0821495618025339  | 1.731365157320742
0.9433702345510857  | 1.7018085726432681
0.6166144981519759  | 1.6562709728670864
0.4791743590718513  | 1.434042150748819
0.4774487755362498  | 1.3777334477362078

10/30 - trial 3
      Initial       |        Final
------------------- | -------------------
0.7990966560407698  | 1.5627371018573175
0.6204843766200729  | 1.3558589065645898
0.5054457172276849  | 1.3411313616328282
0.5042630912038997  | 1.3011877406074022
0.38990186075698696 | 1.2642438490110357

From the above results, we can see that having more generations definitely stabilizes the fitness of the creature, but this also means the fitness plateaus. with more children and fewer generations, despite having a larger variance, the fitness levels also increase quite steadily. it might be more helpful to prioritize evolution with quantity of specimen over generations of mutation. 