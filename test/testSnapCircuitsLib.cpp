#define NANOSVG_IMPLEMENTATION      // Expands implementation
#define NANOSVGRAST_IMPLEMENTATION  // Expands implementation

#include "snapCircuits/utils.h"
#include "snapCircuits/snapCircuitsPart.h"
#include "snapCircuits/snapCircuitsBoard.h"

#include <gtest/gtest.h>

#include <stdio.h>

using namespace std;
using namespace snapCircuits;

TEST(snapLocation, testClassIntegrity)
{
    snapLocation loc;

    EXPECT_EQ(-1, loc.getX());
    EXPECT_EQ(-1, loc.getY());
}

TEST(snapCircuitsPart, testClassIntegrity)
{
    string label="BG";
    snapCircuitsPart p(label);

    EXPECT_EQ(label, p.getLabel());
}

TEST(snapCircuitsBoard, testClassIntegrity)
{

    snapCircuitsPart spPart("WC",snapLocation(7,1,360,10,8));

    snapCircuitsBoard board;
    board.addPart(spPart);
    board.addPart(snapCircuitsPart("WC",snapLocation(1,1,90)));
    
    EXPECT_EQ(true, board.removePart(1));
    
    board.addPart(snapCircuitsPart("WC",snapLocation(2,4,90)));
    board.addPart(snapCircuitsPart("WC",snapLocation(1,5,270)));
    board.addPart(snapCircuitsPart("WC",snapLocation(3,3,180)));
    
    EXPECT_NE(true, board.removePart(10));
    EXPECT_EQ(true, board.removePart(3));

    snapCircuitsBoard board1;
    board1=board;

    EXPECT_EQ(true, board1==board);
    board1.reset();
    EXPECT_NE(true, board1==board);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
