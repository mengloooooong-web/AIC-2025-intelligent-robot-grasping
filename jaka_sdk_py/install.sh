
# 当前pyhton的dist-packages路径
python_path=$(python3 -c "import site; print(site.getsitepackages()[0])")
install()
{
    echo "Installing SDK for Python"
    echo "more info: https://www.jaka.com/docs/guide/SDK/Python.html"
    sudo cp ./lib/jkrc.so ${python_path}
    sudo cp ./lib/jkrc.so /usr/lib
    sudo cp ./lib/libjakaAPI.so /usr/lib
}

uninstall()
{
    echo "Uninstalling SDK for Python"
    sudo rm ${python_path}/jkrc.so
    sudo rm /usr/lib/jkrc.so
    sudo rm /usr/lib/libjakaAPI.so
}

if [ "$1" = "install" ]; then
    install
elif [ "$1" = "uninstall" ]; then
    uninstall
else
    echo "Usage: $0 {install|uninstall}"
fi