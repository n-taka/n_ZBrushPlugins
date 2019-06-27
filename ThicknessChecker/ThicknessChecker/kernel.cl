OCLSTRINGIFY(
__kernel void test1(__global int* result, __private int sizeX)
{
    int x = get_global_id(0);
    int y = get_global_id(1);

    result[sizeX * y + x] = (x + y);
}
__kernel void test2(__global int* result, __private int sizeX)
{
    int x = get_global_id(0);
    int y = get_global_id(1);

    result[sizeX * y + x] = (x - y);
}
)
