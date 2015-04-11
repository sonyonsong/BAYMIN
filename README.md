# Lessons in Hating the CLIC Lab: Installing Wit Locally

Download the library source:
1. `openssl`
2. `curl`
3. `sox`

## Building Libraries

Choose a directory to store the library files. In this case, we use
`/home/esl2131/wit`.

### OpenSSL

```bash
./configure shared --prefix=/home/esl2131/wit
make && make install
```

### Curl

```
./configure --with-ssl=/home/esl2131/wit --prefix=/home/esl2131
make && make install
```

### Sox

```
./configure --prefix=/home/esl2131
make && make install
```

### Wit

Download the Python framework. Add your directory to the `include_dirs` and
`library_dirs` arguments. The include should be the directory above, with the
`/include` subdirectory and the library should be with the `/library`
directory. Then run normal instructions, skipping the install step.

Move the generated `wit.so` file to somewhere convenient, and include this
path in your `sys.path` by appending it. You should now be able to simply
`import wit` to use it.
