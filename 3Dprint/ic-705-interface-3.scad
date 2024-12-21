/*
    IC-705 interface PCB box
    by OK1HRA, Creative Commons BY-SA
 -------------------------------------------------------------------
     How to export .stl
     After open press F5 for preview
     .STL export press F6 and F7 (or menu /Design/Export as STL...)

*/
mountPoint = 0;         // [0:1] enable mount for screw

//# translate([0,0,0]) rotate([0,0,0]) import("IC-705-interface-PCB-02.stl", convexity=3);

//case();

blob();

module blob(){
    difference(){
        translate([86/2,45/2,-14]) scale([1.2, 0.8, 0.5]) sphere(d=150, $fn=150);
        translate([-60,-50,-100]) cube([200,200,100]);
            translate([0,0,-1.5]) layout(-1.5, -1.5, 13.1+3); 
            translate([0,0,-1.6]) layout(1, 1, 3.2); 
            // LED
            translate([14,1.5,1]) cylinder(h=26, d=5, center=false, $fn=40);  
            translate([18.5,1.5,1]) cylinder(h=26, d=5, center=false, $fn=40);  
            translate([45.5,1.5,1]) cylinder(h=26, d=5, center=false, $fn=40);
            translate([65.5,1.5,1]) cylinder(h=26, d=5, center=false, $fn=40);  
            translate([70.0,1.5,1]) cylinder(h=26, d=5, center=false, $fn=40);  
            // jack
//            hull(){
//                translate([37,3,4.15]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
//                translate([37,3,0]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
//            }
//            hull(){
//                translate([53.5,3,4.15]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);  
//                translate([53.5,3,0]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);  
//            }
//            hull(){
//                translate([69,45,4.15]) rotate([-90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
//                translate([69,45,0]) rotate([-90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
//            }
            // usb
            P=6;
            hull(){
                translate([33.0,45,3.2]) rotate([-90,0,0]) cylinder(h=50, d=3.5+P, center=false, $fn=40);
                    translate([33.0,45,0]) rotate([-90,0,0]) cylinder(h=50, d=3.5+P, center=false, $fn=40);
                translate([26.9,45,3.2]) rotate([-90,0,0]) cylinder(h=50, d=3.5+P, center=false, $fn=40);
                    translate([26.9,45,0]) rotate([-90,0,0]) cylinder(h=50, d=3.5+P, center=false, $fn=40);
            }
            // ETH
//            translate([45.4,40,1.6]) cube([15.7,50,13]);
            // DC jack
//            translate([76.5,20,1.6]) cube([9.5,50,11.3]);
//            translate([75.75,-1,1.6]) cube([9.5,5,11.3]);
    }
    difference(){
        union(){
            hull(){
                translate([18,33,1.6]) cylinder(h=14, d=6, center=false, $fn=40);
                translate([1.0,33-3,1.6]) cube([0.5,6,14]);
            }
            hull(){
                translate([84.5,28.5,1.6]) cylinder(h=13.5, d=6, center=false, $fn=40);  
                translate([86.5,28.5-3,1.6]) cube([0.5,6,14]);
            }

        }
        translate([18,33,1.5]) cylinder(h=9, d2=2.5, d1=3, center=false, $fn=40);  
        translate([84.5,28.5,1.5]) cylinder(h=9, d2=2.5, d1=3, center=false, $fn=40);  
    }

}

module case(){
    difference(){
        union(){
            translate([0,0,1.6+13]) layout(-1, 0, 2); 
            translate([0,0,1.6]) layout(0, 0, 13); 
            if(mountPoint==1){
                hull(){
                    translate([-6,22.5,1.6+12]) cylinder(h=3, d=13, center=false, $fn=40);
                        translate([0,22.5-10,1.6+12]) cube([1,20,3]);
                    translate([86+6+2,22.5,1.6+12]) cylinder(h=3, d=13, center=false, $fn=40);
                        translate([86+1,22.5-10,1.6+12]) cube([1,20,3]);
                }
            }
        }
        if(mountPoint==1){
            translate([-6,22.5,0]) cylinder(h=20, d=5, center=false, $fn=40);
//                translate([-6,22.5,4.6]) cylinder(h=15, d=11, center=false, $fn=40);
            translate([86+6+2,22.5,0]) cylinder(h=20, d=5, center=false, $fn=40);
//                translate([86+6+2,22.5,4.6]) cylinder(h=15, d=11, center=false, $fn=40);
        }
        // in
        difference(){
            translate([0,0,1.5]) layout(-1.5, -1.5, 13.1); 
            // LED
            translate([14,1.5,1]) cylinder(h=16, d=5, center=false, $fn=40);  
            translate([18.5,1.5,1]) cylinder(h=16, d=5, center=false, $fn=40);  
            translate([45.5,1.5,1]) cylinder(h=16, d=5, center=false, $fn=40);
            translate([65.5,1.5,1]) cylinder(h=16, d=5, center=false, $fn=40);  
            translate([70.0,1.5,1]) cylinder(h=16, d=5, center=false, $fn=40);  
        }
        // jack
        hull(){
            translate([37,45,4.15]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
            translate([37,45,0]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
        }
        hull(){
            translate([53.5,45,4.15]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);  
            translate([53.5,45,0]) rotate([90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);  
        }
        hull(){
            translate([69,45,4.15]) rotate([-90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
            translate([69,45,0]) rotate([-90,0,0]) cylinder(h=50, d=5.2, center=false, $fn=40);
        }
        // usb
        hull(){
            translate([33.0,45,3.2]) rotate([-90,0,0]) cylinder(h=50, d=3.5, center=false, $fn=40);
                translate([33.0,45,0]) rotate([-90,0,0]) cylinder(h=50, d=3.5, center=false, $fn=40);
            translate([26.9,45,3.2]) rotate([-90,0,0]) cylinder(h=50, d=3.5, center=false, $fn=40);
                translate([26.9,45,0]) rotate([-90,0,0]) cylinder(h=50, d=3.5, center=false, $fn=40);
        }
        // ETH
        translate([45.4,20,1.6]) cube([15.7,50,13]);
        // DC jack
        translate([76.5,20,1.6]) cube([9.5,50,11.3]);
        translate([75.75,-1,1.6]) cube([9.5,5,11.3]);
        // LED
        ledeye(14);
        ledeye(18.5);
        ledeye(45.5);
        ledeye(65.5);
        ledeye(70.0);
        rotate([0,0,0]) translate([43,18,16.6-0.4]){
            linear_extrude(height = 3, center = false, convexity = 5, twist = -0, slices = 20, scale = 1.0) {
                text(str("IC-705"), font = "Sans Uralic:style=Bold", halign="center", size=width/16);
            }
        }
    }
    difference(){
        union(){
            hull(){
                translate([18,33,1.6]) cylinder(h=14, d=6, center=false, $fn=40);
                translate([1.0,33-3,1.6]) cube([0.5,6,14]);
            }
            hull(){
                translate([84.5,28.5,1.6]) cylinder(h=13.5, d=6, center=false, $fn=40);  
                translate([86.5,28.5-3,1.6]) cube([0.5,6,14]);
            }

        }
        translate([18,33,1.5]) cylinder(h=9, d2=2.5, d1=3, center=false, $fn=40);  
        translate([84.5,28.5,1.5]) cylinder(h=9, d2=2.5, d1=3, center=false, $fn=40);  
    }
}

module ledeye(XX){
    hull(){
        translate([XX,1.5,1]) rotate([14,0,0]) cylinder(h=16, d=3, center=false, $fn=40);
        translate([XX,-1.5,1]) rotate([14,0,0]) cylinder(h=16, d=4, center=false, $fn=40);
    }
    translate([XX,5,1.8]) rotate([90,0,0]) cylinder(h=16, d=3, center=false, $fn=40);
}


module layout(SHIFTT, SHIFTB, ZZ){
    hull(){
        translate([2,2,0]) cylinder(h=ZZ, d2=4+2*SHIFTT, d1=4+2*SHIFTB, center=false, $fn=40);  
        translate([2,45,0]) cylinder(h=ZZ, d2=4+2*SHIFTT, d1=4+2*SHIFTB, center=false, $fn=40);  
        translate([86,2,0]) cylinder(h=ZZ, d2=4+2*SHIFTT, d1=4+2*SHIFTB, center=false, $fn=40);  
        translate([86,45,0]) cylinder(h=ZZ, d2=4+2*SHIFTT, d1=4+2*SHIFTB, center=false, $fn=40);  
    }
}