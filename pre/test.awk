BEGIN{
     pos = "3dpos.txt";
}
{
    print($3 " " $4 " " $5 ) > pos;
}
END{
fflush(pos);
close(pos)
}
