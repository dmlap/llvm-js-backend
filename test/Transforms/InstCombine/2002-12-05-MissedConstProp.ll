; RUN: as < %s | opt -instcombine | dis | grep-not add

int %test(int %A) {
  %A.neg = sub int 0, %A
  %.neg = sub int 0, 1
  %X = add int %.neg, 1
  %Y.neg.ra = add int %A, %X
  %r = add int %A.neg, %Y.neg.ra
  ret int %r
}
