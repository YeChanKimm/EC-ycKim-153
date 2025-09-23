// /*-------------------------------------------------------------------------------\
// @ c-tutorial by young-keun kim - handong global university
// author           : sss lab
// created          : 19-08-2022
// modified         : 19-08-2022
// language/ver     : c in msvs2022
// description      : c_structure_exercise2.c
// -------------------------------------------------------------------------------*/

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

// #define _crt_secure_no_warnings

// typedef struct {
//   char building_name[100];
//   int room_number;
//   char room_name[100];
// } handong;


// int main()
// {
//   // create structure type (handong)  variable  room1
//   handong room1;

//   // create structure type (handong)  variable  room2, room3
//   handong room2;
//   handong room3;


//   // create structure type (handong)  pointer , roompt
//   handong* room1pt = &room1;

//   // assign room3 address to roompt
//   handong* room2pt = &room2;
//   handong* room3pt = &room3;


//   // define structure variable  memeber values: room1
//   strcpy_s(room1.building_name, sizeof(char) * 10, "newton" );
//   room1.room_number = 109;
//   strcpy_s(room1.room_name, sizeof(char) * 10, "iilab");

//   // define structure variable  memeber values: room2
//   strcpy_s(room2.building_name, sizeof(char) * 10, "rodem");
//   room2.room_number = 323;
//   strcpy_s(room2.room_name, sizeof(char) * 10, "hello");

//   // define structure variable  memeber values: room3
//   strcpy_s(room3.building_name, sizeof(char) * 10, "osuk");
//   room3.room_number = 406;
//   strcpy_s(room3.room_name, sizeof(char) * 10,"world");



//   // print each member values  : room1, room2, room3
//   printf("%s building, room  %d  is %s\n", room1.building_name, room1.room_number, room1.room_name);
//   printf("%s building, room  %d  is %s\n", room2.building_name, room2.room_number, room2.room_name);
//   printf("%s building, room  %d  is %s\n", room3.building_name, room3.room_number, room3.room_name);

//   // print each member values by  pointer variable: room1pt
//   putchar('\n');
  
//   printf("%s building, room  %d  is %s\n", room1pt->building_name, room1pt->room_number, room1pt->room_name);
//   printf("%s building, room  %d  is %s\n", room2pt->building_name, room2pt->room_number, room2pt->room_name);
//   printf("%s building, room  %d  is %s\n", room3pt->building_name, room3pt->room_number, room3pt->room_name);

//   // print address of   room1  and value of room1pt and compare. 
//   printf("\n room1 address=%x ,  roompt = %x  \n", &room1, room1pt);


// }
