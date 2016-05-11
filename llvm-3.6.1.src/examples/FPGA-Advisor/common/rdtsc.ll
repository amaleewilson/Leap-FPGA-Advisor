; ModuleID = 'rdtsc.ll'
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; Function Attrs: nounwind uwtable
define i64 @get_rdtsc() #0 {
entry:
  %hi = alloca i32, align 4
  %lo = alloca i32, align 4
  %0 = call { i32, i32 } asm sideeffect "rdtsc", "={ax},={dx},~{dirflag},~{fpsr},~{flags}"() #1, !srcloc !1
  %asmresult = extractvalue { i32, i32 } %0, 0
  %asmresult1 = extractvalue { i32, i32 } %0, 1
  store i32 %asmresult, i32* %lo, align 4
  store i32 %asmresult1, i32* %hi, align 4
  %1 = load i32* %lo, align 4
  %conv = zext i32 %1 to i64
  %2 = load i32* %hi, align 4
  %conv2 = zext i32 %2 to i64
  %shl = shl i64 %conv2, 32
  %or = or i64 %conv, %shl
  ret i64 %or
}

attributes #0 = { nounwind uwtable "less-precise-fpmad"="false" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nounwind }

!llvm.ident = !{!0}

!0 = !{!"clang version 3.6.1 (tags/RELEASE_361/final)"}
!1 = !{i32 111}
